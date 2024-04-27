// See LICENSE.Berkeley for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.lsu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._

import boom.common._
import boom.exu.CommitSignals

class SelectedPrediction(implicit p : Parameters) extends BoomBundle()(p) {
  val lane = UInt(log2Ceil(coreWidth).W)
  val addr = UInt(coreMaxAddrBits.W)
  val same_page = Bool()
  val confidence = UInt(4.W)
}

class Prediction(implicit p : Parameters) extends BoomBundle()(p) {
  val addr = UInt(coreMaxAddrBits.W)
  val same_page = Bool()
  val confidence = UInt(4.W)
}

// A predictor entry
class Entry(implicit p : Parameters) extends BoomBundle()(p) {
  val previous = UInt(coreMaxAddrBits.W)
  val confidence = UInt(4.W)
  val stride = UInt(coreMaxAddrBits.W)
}

class RfpLSUIO(implicit p: Parameters) extends BoomBundle()(p) {
  val ldq_idx     = Output(Vec(coreWidth, Valid(UInt(ldqAddrSz.W))))
  val predictions = Output(Vec(coreWidth, Valid(new Prediction())))
  // We output a single prediction
  val prediction  = Output(Valid(new SelectedPrediction()))
  // To verify matching PCs
  val debug_uops  = Output(Vec(coreWidth, Valid(new MicroOp())))
  val ldq_entry   = Input(Vec(coreWidth, new LDQEntry))
}

// Currently only does training, detecting strided accesses
// TODO: Actually use the optimised bank selection
//       Likely not inferring optimal number of ports (1w2r)
// TODO: Make banks set-associative
class Rfp(implicit p: Parameters) extends BoomModule()(p) {
  val io = IO(new Bundle {
    val lsu = new RfpLSUIO
    val commit = Input(new CommitSignals)
    // Incoming uops to make predictions for
    val enq_uops = Input(Vec(coreWidth, Valid(new MicroOp())))
  })

  val nEntries = 64
  val associativity = 1
  // Ensure each bank is a power of two
  require(isPow2(nEntries/associativity))

  val nBanks = if (coreWidth == 1) 1 else 2 << log2Ceil(coreWidth)
  val nBankBits = log2Ceil(nBanks)

  val nEntriesPerBank = nEntries/nBanks
  val nIndexBits = log2Ceil(nEntriesPerBank)

  // bit (1) is part of the tag instead of the index so that we can handle compressed instructions
  private def getTag(addr: UInt): UInt = addr(coreMaxAddrBits - 1, nBankBits + nIndexBits + 1)
  private def getIdx(addr: UInt): UInt = addr(nBankBits + nIndexBits, nBankBits + 1)
  // getBank gets the bank for a given instruction address and guarantees that up to `coreWidth` consecutive instruction addresses land in different banks
  private def getBank(addr: UInt): UInt = if (coreWidth == 1) 0.U else Cat(addr(nBankBits + 1) ^ addr(1), addr(nBankBits, 2))
  private def saturatingIncrement(confidence: UInt, max: UInt): UInt = Mux(confidence =/= max, confidence + 1.U, max) 

  // The memory of the predictor is divided into banks
  val mem  = Reg(Vec(nBanks, Vec(nEntriesPerBank, new Entry())))
  // TODO: Init this properly
  val tags = Reg(Vec(nBanks, Vec(nEntriesPerBank, Valid(UInt((coreMaxAddrBits - nBankBits - nIndexBits - 1).W)))))

  // === Train the predictor on committing loads ===
  for (w <- 0 until coreWidth) {
    io.lsu.ldq_idx(w).valid := false.B
    io.lsu.ldq_idx(w).bits := 0.U
    
    val uop = io.commit.uops(w)

    val pc = uop.debug_pc
    val ldq_idx = uop.ldq_idx

    val idx = getIdx(pc)
    val tag = getTag(pc)
    val bank = getBank(pc)

    io.lsu.ldq_idx(w).valid := io.commit.valids(w)
    io.lsu.ldq_idx(w).bits := ldq_idx
    val addr = io.lsu.ldq_entry(w).addr.bits

    val hit = tags(bank)(idx).valid && (tags(bank)(idx).bits === tag)

    val previous = mem(bank)(idx).previous
    val stride = mem(bank)(idx).stride
    val confidence = mem(bank)(idx).confidence

    val new_stride = addr - previous
    val matches = (previous + stride === addr) && (new_stride === stride) && hit
    val new_confidence = Mux(matches, saturatingIncrement(confidence, 15.U), 0.U)

    val commit_valid = io.commit.valids(w)
    val commit_uop = io.commit.uops(w)
    val ldq_e = io.lsu.ldq_entry(w)

    when (commit_valid && commit_uop.uses_ldq && !commit_uop.uses_stq && !ldq_e.addr_is_uncacheable) {
      tags(bank)(idx).bits := tag
      tags(bank)(idx).valid := true.B

      mem(bank)(idx).previous   := addr
      mem(bank)(idx).stride     := new_stride
      mem(bank)(idx).confidence := new_confidence
    }
  }

  // === Generate predictions on dispatched loads ===
  val predictions = Reg(Vec(coreWidth, Valid(new Prediction())))
  val debug_uops = Reg(Vec(coreWidth, Valid(new MicroOp())))

  io.lsu.predictions := predictions
  io.lsu.debug_uops := debug_uops

  for (w <- 0 until coreWidth) {
    // Try to generate predictions for each incoming instruction
    val prediction_uop = io.enq_uops(w).bits
    val prediction_pc = prediction_uop.debug_pc
    val prediction_idx = getIdx(prediction_pc)
    val prediction_tag = getTag(prediction_pc)
    val prediction_bank = getBank(prediction_pc)
    val prediction_hit = tags(prediction_bank)(prediction_idx).valid && (tags(prediction_bank)(prediction_idx).bits === prediction_tag) && io.enq_uops(w).valid
    val prediction_prev = mem(prediction_bank)(prediction_idx).previous
    val prediction_stride = mem(prediction_bank)(prediction_idx).stride
    val prediction_confidence = mem(prediction_bank)(prediction_idx).confidence
    val prediction_addr = WireInit(prediction_prev + prediction_stride)

    // compare the page/frame numbers
    // true when the prediction_addr is in the same page as the previously accessed value
    val prediction_same_page = prediction_prev(coreMaxAddrBits - 1, 12) === prediction_addr(coreMaxAddrBits - 1, 12)

    // prediction_addr is only valid when it's within the same page
    val prediction_valid = ( true.B
      && prediction_hit 
      && prediction_same_page 
      && (prediction_confidence >= 8.U)
      && (prediction_pc =/= RegNext(prediction_pc)) // Don't issue prediction for same instruction two cycles in a row.
    )

    predictions(w).valid := prediction_valid
    predictions(w).bits.confidence := prediction_confidence
    predictions(w).bits.same_page := prediction_same_page
    predictions(w).bits.addr := prediction_addr
    debug_uops(w).valid := prediction_valid
    debug_uops(w).bits := prediction_uop
  }

  // TODO: This is really ugly code, but my Scala-fu is too weak to do anything about it
  val select = predictions.reduce((a, b) => {
    val sel = (a.valid && b.valid && a.bits.confidence >= b.bits.confidence) || !b.valid
    Mux(sel, a, b)
  })

  val lane = predictions.indexWhere((p: Valid[Prediction]) => {
    p.valid === select.valid && p.bits.confidence === select.bits.confidence && p.bits.addr === select.bits.addr
  })

  val prediction = predictions(lane)
  io.lsu.prediction.valid := prediction.valid
  io.lsu.prediction.bits.addr := prediction.bits.addr
  io.lsu.prediction.bits.confidence := prediction.bits.confidence
  io.lsu.prediction.bits.same_page := prediction.bits.same_page
  io.lsu.prediction.bits.lane := lane
}
