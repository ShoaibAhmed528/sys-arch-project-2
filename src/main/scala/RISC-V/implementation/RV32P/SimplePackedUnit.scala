package RISCV.implementation.RV32P

import chisel3._
import chisel3.util._

import RISCV.interfaces.generic.AbstractExecutionUnit
import RISCV.model._

class SimplePackedUnit extends AbstractExecutionUnit {

  io.misa := "b01__0000__0_00000_00000_00000_10000_00000".U

  // we write the default values for every output port.
  val valid_instr = VecInit(InstructionSets.BasicPacked.map(_.asUInt).toSeq)
  io.valid := valid_instr.contains(io.instr_type.asUInt)
  io.stall := STALL_REASON.NO_STALL
  io_reg.reg_rs1 := 0.U // for immediate instruc, we set rs1 and rs2 to 0.
  io_reg.reg_rs2 := 0.U
  io_reg.reg_rd := io.instr(11, 7)
  io_reg.reg_write_en := false.B
  io_reg.reg_write_data := 0.U
  io_pc.pc_we := false.B
  io_pc.pc_wdata := 0.U
  io_data.data_req := false.B
  io_data.data_addr := 0.U
  io_data.data_be := 0.U
  io_data.data_we := false.B
  io_data.data_wdata := 0.U
  io_trap.trap_valid := false.B
  io_trap.trap_reason := TRAP_REASON.NONE

  val instr = io.instr
  val instr_type = io.instr_type
  val rs1 = io_reg.reg_read_data1
  val rs2 = io_reg.reg_read_data2


  // pli.b. 8-bit imm[23:16]
  val pli_b_imm = instr(23, 16)

  // pli.h.10-bit signed imm.
  val pli_h_imm_raw = Cat(instr(15), instr(24, 16))   // we concatenate sign bit with rest.
  val pli_h_imm16   = Cat(Fill(6, pli_h_imm_raw(9)), pli_h_imm_raw) // then we extend it.

  // plui.h. 10-bit imm that we shift left by 6 bits.
  val plui_h_imm_raw = Cat(instr(23, 15), instr(24))   // {imm[9:1], imm[0]}
  val plui_h_imm16   = Cat(plui_h_imm_raw, 0.U(6.W)) // here we do the shifting.

// we do slicing of registers acc to SIMD.

//byte slicing
  val rs1_b = VecInit.tabulate(4)(i => rs1(i * 8 + 7, i * 8))
  val rs2_b = VecInit.tabulate(4)(i => rs2(i * 8 + 7, i * 8))
//halfword slicing
  val rs1_h = VecInit.tabulate(2)(i => rs1(i * 16 + 15, i * 16))
  val rs2_h = VecInit.tabulate(2)(i => rs2(i * 16 + 15, i * 16))

  //padd.b
  val padd_b = VecInit.tabulate(4)(i => (rs1_b(i) +& rs2_b(i))(7, 0))
  //padd.h
  val padd_h = VecInit.tabulate(2)(i => (rs1_h(i) +& rs2_h(i))(15, 0))
//padd.bs
  val padd_bs_scalar = rs2(7, 0)
  val padd_bs = VecInit.tabulate(4)(i => (rs1_b(i) +& padd_bs_scalar)(7, 0))
//padd.hs
  val padd_hs_scalar = rs2(15, 0)
  val padd_hs = VecInit.tabulate(2)(i => (rs1_h(i) +& padd_hs_scalar)(15, 0))
//psub.b
  val psub_b = VecInit.tabulate(4)(i => (rs1_b(i) -& rs2_b(i))(7, 0))
//psub.h
  val psub_h = VecInit.tabulate(2)(i => (rs1_h(i) -& rs2_h(i))(15, 0))

  //psadd (saturating signed add)
  val psadd_b = VecInit.tabulate(4) { i =>
    val sum = rs1_b(i).asSInt +& rs2_b(i).asSInt             // 9-bit signed
    val sat = Mux(sum > 127.S(9.W), 127.S(9.W),
              Mux(sum < -128.S(9.W), -128.S(9.W), sum))
    sat(7, 0).asUInt
  }

  val psadd_h = VecInit.tabulate(2) { i =>
    val sum = rs1_h(i).asSInt +& rs2_h(i).asSInt             // 17-bit signed
    val sat = Mux(sum > 32767.S(17.W), 32767.S(17.W),
              Mux(sum < -32768.S(17.W), -32768.S(17.W), sum))
    sat(15, 0).asUInt
  }

  //paadd (averaging signed add)
  val paadd_b = VecInit.tabulate(4) { i =>
    val sum = rs1_b(i).asSInt +& rs2_b(i).asSInt             // 9-bit signed
    sum(8, 1).asUInt                                          // right shift 1
  }

  val paadd_h = VecInit.tabulate(2) { i =>
    val sum = rs1_h(i).asSInt +& rs2_h(i).asSInt             // 17-bit signed
    sum(16, 1).asUInt
  }

  // Finally we do the writing back.
  when(io.valid) {
    io_pc.pc_we    := true.B
    io_pc.pc_wdata := io_pc.pc + 4.U
    io_reg.reg_rs1 := Mux(instr_type === RISCV_TYPE.pli_b || instr_type === RISCV_TYPE.pli_h || instr_type === RISCV_TYPE.plui_h, 0.U, io.instr(19, 15))
    io_reg.reg_rs2 := Mux(instr_type === RISCV_TYPE.pli_b || instr_type === RISCV_TYPE.pli_h || instr_type === RISCV_TYPE.plui_h, 0.U, io.instr(24, 20))
    io_reg.reg_write_en := true.B

    switch(instr_type) {
      is(RISCV_TYPE.pli_b) {
        io_reg.reg_write_data := Cat(pli_b_imm, pli_b_imm, pli_b_imm, pli_b_imm)
      }
      is(RISCV_TYPE.pli_h) {
        io_reg.reg_write_data := Cat(pli_h_imm16, pli_h_imm16)
      }
      is(RISCV_TYPE.plui_h) {
        io_reg.reg_write_data := Cat(plui_h_imm16, plui_h_imm16)
      }
      is(RISCV_TYPE.padd_b) {
        io_reg.reg_write_data := Cat(padd_b(3), padd_b(2), padd_b(1), padd_b(0))
      }
      is(RISCV_TYPE.padd_h) {
        io_reg.reg_write_data := Cat(padd_h(1), padd_h(0))
      }
      is(RISCV_TYPE.padd_bs) {
        io_reg.reg_write_data := Cat(padd_bs(3), padd_bs(2), padd_bs(1), padd_bs(0))
      }
      is(RISCV_TYPE.padd_hs) {
        io_reg.reg_write_data := Cat(padd_hs(1), padd_hs(0))
      }
      is(RISCV_TYPE.psub_b) {
        io_reg.reg_write_data := Cat(psub_b(3), psub_b(2), psub_b(1), psub_b(0))
      }
      is(RISCV_TYPE.psub_h) {
        io_reg.reg_write_data := Cat(psub_h(1), psub_h(0))
      }
      is(RISCV_TYPE.psadd_b) {
        io_reg.reg_write_data := Cat(psadd_b(3), psadd_b(2), psadd_b(1), psadd_b(0))
      }
      is(RISCV_TYPE.psadd_h) {
        io_reg.reg_write_data := Cat(psadd_h(1), psadd_h(0))
      }
      is(RISCV_TYPE.paadd_b) {
        io_reg.reg_write_data := Cat(paadd_b(3), paadd_b(2), paadd_b(1), paadd_b(0))
      }
      is(RISCV_TYPE.paadd_h) {
        io_reg.reg_write_data := Cat(paadd_h(1), paadd_h(0))
      }
    }
  }
}