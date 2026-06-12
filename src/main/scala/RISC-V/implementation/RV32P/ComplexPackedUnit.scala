package RISCV.implementation.RV32P

import chisel3._
import chisel3.util._
import RISCV.interfaces.generic.AbstractExecutionUnit
import RISCV.model.InstructionSets
import RISCV.model.STALL_REASON
import RISCV.model.TRAP_REASON
import RISCV.model.RISCV_TYPE

class ComplexPackedUnit extends AbstractExecutionUnit {
  io.misa := "b01__0000__0_00000_00000_00000_10000_00000".U

  // Claim all ComplexPacked instructions
  val valid_instr = VecInit(InstructionSets.ComplexPacked.map(_.asUInt).toSeq)
  io.valid := valid_instr.contains(io.instr_type.asUInt)

  // Default outputs
  io.stall := STALL_REASON.NO_STALL
  io_pc.pc_we := false.B
  io_pc.pc_wdata := io_pc.pc + 4.U
  io_reg.reg_rs1 := io.instr(19, 15)
  io_reg.reg_rs2 := io.instr(24, 20)
  io_reg.reg_rd := io.instr(11, 7)
  io_reg.reg_write_en := false.B
  io_reg.reg_write_data := 0.U
  io_data.data_req := false.B
  io_data.data_addr := 0.U
  io_data.data_be := 0.U
  io_data.data_we := false.B
  io_data.data_wdata := 0.U
  io_trap.trap_valid := false.B
  io_trap.trap_reason := TRAP_REASON.NONE

  val rs1 = io_reg.reg_read_data1
  val rs2 = io_reg.reg_read_data2

  // signed shamt from rs2 low byte (for shift instructions)
  val sshamt = rs2(7, 0).asSInt

  // shamt immediates
  val shamt_i5 = io.instr(24, 20)
  val shamt_i4 = io.instr(23, 20)

  // here are the helpers 

  def clipS16(x: SInt): UInt = {
    Mux(
      x < (-32768).S,
      0x8000.U(16.W),
      Mux(x > 32767.S, 0x7fff.U(16.W), x(15, 0))
    )
  }

  def clipU16(x: UInt): UInt =
    Mux(x > 0xffff.U, 0xffff.U(16.W), x(15, 0))

  def clipS8(x: SInt): UInt = {
    Mux(x < (-128).S, 0x80.U(8.W), Mux(x > 127.S, 0x7f.U(8.W), x(7, 0)))
  }

  def clipS32(x: SInt): UInt = {
    val minV = (-2147483648L).S(65.W)
    val maxV = 2147483647.S(65.W)
    val ext = x.pad(65)
    Mux(
      ext < minV,
      "h80000000".U(32.W),
      Mux(ext > maxV, "h7fffffff".U(32.W), x(31, 0))
    )
  }

  //PSSHA.HS
  val pssha_lanes = VecInit((0 until 2).map { i =>
    val h = rs1(16 * i + 15, 16 * i).asSInt
    val r = Wire(UInt(16.W))
    when(sshamt < 0.S) {
      val neg = (-sshamt).asUInt
      when(neg >= 16.U) { r := Mux(h < 0.S, 0xffff.U, 0.U) }
        .otherwise { r := (h >> neg).asUInt(15, 0) }
    }.otherwise {
      r := clipS16((h.pad(32) << sshamt.asUInt).asSInt)
    }
    r
  })
  val pssha_result = pssha_lanes(1) ## pssha_lanes(0)

  //PSSHAR.HS
  val psshar_lanes = VecInit((0 until 2).map { i =>
    val h = rs1(16 * i + 15, 16 * i).asSInt
    val r = Wire(UInt(16.W))
    when(sshamt < 0.S) {
      val neg = (-sshamt).asUInt.min(16.U)
      val shifted = (h >> neg).asSInt 
      val round = Mux(neg > 0.U, (h >> (neg - 1.U))(0), 0.U)
      r := (shifted + round.asSInt)(15, 0)
    }.otherwise {
      r := clipS16((h.pad(32) << sshamt.asUInt).asSInt)
    }
    r
  })
  val psshar_result = psshar_lanes(1) ## psshar_lanes(0)

  val psshl_lanes = VecInit((0 until 2).map { i =>
    val h = rs1(16 * i + 15, 16 * i)
    val r = Wire(UInt(16.W))
    when(sshamt < 0.S) {
      val neg = (-sshamt).asUInt
      when(neg >= 16.U) { r := 0.U }
        .otherwise { r := (h >> neg)(15, 0) }
    }.otherwise {
      r := clipU16(h << sshamt.asUInt)
    }
    r
  })
  val psshl_result = psshl_lanes(1) ## psshl_lanes(0)

  
  val psshlr_lanes = VecInit((0 until 2).map { i =>
    val h = rs1(16 * i + 15, 16 * i)
    val r = Wire(UInt(16.W))
    when(sshamt < 0.S) {
      val neg = (-sshamt).asUInt.min(16.U)
      val shifted = h >> neg
      val round = Mux(neg > 0.U, (h >> (neg - 1.U))(0), 0.U)
      r := (shifted + round)(15, 0)
    }.otherwise {
      r := clipU16(h << sshamt.asUInt)
    }
    r
  })
  val psshlr_result = psshlr_lanes(1) ## psshlr_lanes(0)

  // PNCLIPI.B / PNCLIPRI.B
  // {rs2, rs1} = 64-bit source, 4 × 16-bit lanes → clip to signed 8-bit
  val pnclipi_b_lanes = VecInit((0 until 4).map { i =>
    val word = if (i < 2) rs1 else rs2
    val h = word(16 * (i % 2) + 15, 16 * (i % 2)).asSInt
    clipS8((h >> shamt_i4).asSInt)
  })
  val pnclipi_b_result = pnclipi_b_lanes(3) ## pnclipi_b_lanes(2) ##
    pnclipi_b_lanes(1) ## pnclipi_b_lanes(0)

  val pnclipri_b_lanes = VecInit((0 until 4).map { i =>
    val word = if (i < 2) rs1 else rs2
    val h = word(16 * (i % 2) + 15, 16 * (i % 2)).asSInt
    val shifted = (h >> shamt_i4).asSInt
    val round = Mux(shamt_i4 > 0.U, (h >> (shamt_i4 - 1.U))(0), 0.U)
    clipS8((shifted + round.asSInt).asSInt)
  })
  val pnclipri_b_result = pnclipri_b_lanes(3) ## pnclipri_b_lanes(2) ##
    pnclipri_b_lanes(1) ## pnclipri_b_lanes(0)

  //PNCLIPI.H / PNCLIPRI.H
  // {rs2, rs1} = 64-bit source, 2 × 32-bit lanes → clip to signed 16-bit
  val pnclipi_h_lanes = VecInit((0 until 2).map { i =>
    val w = (if (i == 0) rs1 else rs2).asSInt
    clipS16((w >> shamt_i5).asSInt)
  })
  val pnclipi_h_result = pnclipi_h_lanes(1) ## pnclipi_h_lanes(0)

  val pnclipri_h_lanes = VecInit((0 until 2).map { i =>
    val w = (if (i == 0) rs1 else rs2).asSInt
    val shifted = (w >> shamt_i5).asSInt
    val round = Mux(shamt_i5 > 0.U, (w >> (shamt_i5 - 1.U))(0), 0.U)
    clipS16((shifted + round.asSInt).asSInt)
  })
  val pnclipri_h_result = pnclipri_h_lanes(1) ## pnclipri_h_lanes(0)

  // PM2ADD family
  def s16(x: UInt): SInt = x(15, 0).asSInt.pad(64)
  def u16(x: UInt): SInt = x(15, 0).zext.pad(64)

  val lo1 = rs1(15, 0); val hi1 = rs1(31, 16)
  val lo2 = rs2(15, 0); val hi2 = rs2(31, 16)

  val pm2add_h_result = (s16(lo1) * s16(lo2) + s16(hi1) * s16(hi2))(31, 0)
  val pm2addsu_h_result = (s16(lo1) * u16(lo2) + s16(hi1) * u16(hi2))(31, 0)
  val pm2addu_h_result = (u16(lo1) * u16(lo2) + u16(hi1) * u16(hi2))(31, 0)
  val pm2add_hx_result = (s16(lo1) * s16(hi2) + s16(hi1) * s16(lo2))(31, 0)

  val pm2sadd_h_result = clipS32(
    (s16(lo1) * s16(lo2) + s16(hi1) * s16(hi2)).asSInt
  )
  val pm2sadd_hx_result = clipS32(
    (s16(lo1) * s16(hi2) + s16(hi1) * s16(lo2)).asSInt
  )

  // Cycle 1: read rd (via rs1 port) + rs2, stall
  // Cycle 2: read rs1 (via rs1 port), compute rd_prev + dot, write back

  val adda_cycle2 = RegInit(false.B)
  val adda_rd_saved = RegInit(0.U(32.W))
  val adda_rs2_saved = RegInit(0.U(32.W))
  val adda_instr_reg = RegInit(0.U(32.W))
  val adda_type_reg = RegInit(RISCV_TYPE.UNKNOWN)

  when(~io_reset.rst_n) {
    adda_cycle2 := false.B
    adda_rd_saved := 0.U
    adda_rs2_saved := 0.U
    adda_instr_reg := 0.U
    adda_type_reg := RISCV_TYPE.UNKNOWN
  }

  val is_pm2adda = VecInit(
    Seq(
      RISCV_TYPE.pm2adda_h,
      RISCV_TYPE.pm2addau_h,
      RISCV_TYPE.pm2addasu_h,
      RISCV_TYPE.pm2adda_hx
    ).map(_.asUInt)
  ).contains(io.instr_type.asUInt)

  // Use saved rs1 (read in cycle 2) and saved rs2 for adda dot
  val adda_rs1 = io_reg.reg_read_data1 // in cycle 2, rs1 port reads actual rs1
  val adda_lo1 = adda_rs1(15, 0); val adda_hi1 = adda_rs1(31, 16)
  val adda_lo2 = adda_rs2_saved(15, 0); val adda_hi2 = adda_rs2_saved(31, 16)

  val adda_dot_ss =
    (s16(adda_lo1) * s16(adda_lo2) + s16(adda_hi1) * s16(adda_hi2))(31, 0)
  val adda_dot_su =
    (s16(adda_lo1) * u16(adda_lo2) + s16(adda_hi1) * u16(adda_hi2))(31, 0)
  val adda_dot_uu =
    (u16(adda_lo1) * u16(adda_lo2) + u16(adda_hi1) * u16(adda_hi2))(31, 0)
  val adda_dot_sx =
    (s16(adda_lo1) * s16(adda_hi2) + s16(adda_hi1) * s16(adda_lo2))(31, 0)

  val adda_dot = MuxLookup(adda_type_reg.asUInt, adda_dot_ss)(
    Seq(
      RISCV_TYPE.pm2adda_h.asUInt -> adda_dot_ss,
      RISCV_TYPE.pm2addasu_h.asUInt -> adda_dot_su,
      RISCV_TYPE.pm2addau_h.asUInt -> adda_dot_uu,
      RISCV_TYPE.pm2adda_hx.asUInt -> adda_dot_sx
    )
  )

  val adda_result = (adda_rd_saved + adda_dot)(31, 0)

  // Main dispatch
  when(io.valid) {
    when(adda_cycle2) {
      // Cycle 2: rs1 port reads actual rs1 (set by default above)
      io_reg.reg_rs1 := adda_instr_reg(19, 15)
      io_reg.reg_rs2 := adda_instr_reg(24, 20)
      io_reg.reg_rd := adda_instr_reg(11, 7)
      io_reg.reg_write_en := true.B
      io_reg.reg_write_data := adda_result
      io_pc.pc_we := true.B
      io_pc.pc_wdata := io_pc.pc + 4.U
      io.stall := STALL_REASON.NO_STALL
      adda_cycle2 := false.B

    }.elsewhen(is_pm2adda) {
      // Cycle 1: read rd via rs1 port, rs2 normally
      io_reg.reg_rs1 := io.instr(11, 7) // rd index → rs1 port
      io_reg.reg_rs2 := io.instr(24, 20)
      adda_rd_saved := io_reg.reg_read_data1
      adda_rs2_saved := io_reg.reg_read_data2
      adda_instr_reg := io.instr
      adda_type_reg := io.instr_type
      io.stall := STALL_REASON.EXECUTION_UNIT
      io_pc.pc_we := false.B
      adda_cycle2 := true.B

    }.otherwise {
      io_reg.reg_rs1 := io.instr(19, 15)
      io_reg.reg_rs2 := io.instr(24, 20)
      io_reg.reg_rd := io.instr(11, 7)
      io_reg.reg_write_en := true.B
      io_pc.pc_we := true.B
      io_pc.pc_wdata := io_pc.pc + 4.U

      io_reg.reg_write_data := MuxLookup(io.instr_type.asUInt, 0.U)(
        Seq(
          RISCV_TYPE.pssha_hs.asUInt -> pssha_result,
          RISCV_TYPE.psshar_hs.asUInt -> psshar_result,
          RISCV_TYPE.psshl_hs.asUInt -> psshl_result,
          RISCV_TYPE.psshlr_hs.asUInt -> psshlr_result,
          RISCV_TYPE.pnclipi_b.asUInt -> pnclipi_b_result,
          RISCV_TYPE.pnclipri_b.asUInt -> pnclipri_b_result,
          RISCV_TYPE.pnclipi_h.asUInt -> pnclipi_h_result,
          RISCV_TYPE.pnclipri_h.asUInt -> pnclipri_h_result,
          RISCV_TYPE.pm2add_h.asUInt -> pm2add_h_result,
          RISCV_TYPE.pm2addsu_h.asUInt -> pm2addsu_h_result,
          RISCV_TYPE.pm2addu_h.asUInt -> pm2addu_h_result,
          RISCV_TYPE.pm2add_hx.asUInt -> pm2add_hx_result,
          RISCV_TYPE.pm2sadd_h.asUInt -> pm2sadd_h_result,
          RISCV_TYPE.pm2sadd_hx.asUInt -> pm2sadd_hx_result
        )
      )
    }
  }
}
