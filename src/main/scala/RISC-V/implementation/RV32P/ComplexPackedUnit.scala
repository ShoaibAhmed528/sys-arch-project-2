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

  val valid_instr = VecInit(InstructionSets.ComplexPacked.map(_.asUInt).toSeq)
  io.valid := valid_instr.contains(io.instr_type.asUInt)

  // 1. Identify clipping instructions upfront for immediate address mapping
  val is_pnclipi_b =
    (io.instr_type.asUInt === RISCV_TYPE.pnclipi_b.asUInt) ||
      (io.instr_type.asUInt === RISCV_TYPE.pnclipri_b.asUInt)
  val is_pnclipi_h =
    (io.instr_type.asUInt === RISCV_TYPE.pnclipi_h.asUInt) ||
      (io.instr_type.asUInt === RISCV_TYPE.pnclipri_h.asUInt)

  // 2. Setup structural defaults
  io.stall := STALL_REASON.NO_STALL
  io_pc.pc_we := false.B
  io_pc.pc_wdata := io_pc.pc + 4.U
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

  // 3. Register address mapping (Moved upfront to fix timing and alignment)
  val encoded_rs1 = io.instr(19, 15)
  when(is_pnclipi_b || is_pnclipi_h) {
    val base_rs1 = encoded_rs1(4, 1) ## 0.U(1.W)
    io_reg.reg_rs1 := base_rs1
    io_reg.reg_rs2 := base_rs1 + 1.U
  }.otherwise {
    io_reg.reg_rs1 := encoded_rs1
    io_reg.reg_rs2 := io.instr(24, 20)
  }

  // 4. Safely sample the updated data streams
  val rs1 = io_reg.reg_read_data1
  val rs2 = io_reg.reg_read_data2

  // Signed shamt from rs2 low byte (for R-type shift instructions)
  val shamt_byte = rs2(7, 0) // raw byte
  val shamt_neg = shamt_byte(7) // sign bit: 1 = negative = right shift
  // magnitude of shift
  val shamt_mag = Mux(shamt_neg, (~shamt_byte + 1.U)(7, 0), shamt_byte)(7, 0)

  // Immediate shamts
  val shamt_i5 = io.instr(24, 20) // 5-bit for pnclipi.h
  val shamt_i4 = io.instr(23, 20) // 4-bit for pnclipi.b

  // clip helpers
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

  val pssha_lanes = VecInit((0 until 2).map { i =>
    val h = rs1(16 * i + 15, 16 * i).asSInt // signed 16-bit lane
    val r = Wire(UInt(16.W))
    when(shamt_neg) {
      // right shift (arithmetic)
      when(shamt_mag >= 16.U) {
        r := Mux(h < 0.S, 0xffff.U, 0.U)
      }.otherwise {
        r := (h >> shamt_mag).asUInt(15, 0)
      }
    }.otherwise {
      // left shift with saturation
      r := clipS16((h.pad(32) << shamt_mag).asSInt)
    }
    r
  })
  val pssha_result = pssha_lanes(1) ## pssha_lanes(0)

  val psshar_lanes = VecInit((0 until 2).map { i =>
    val h = rs1(16 * i + 15, 16 * i).asSInt
    val r = Wire(UInt(16.W))
    when(shamt_neg) {
      val mag = shamt_mag.min(16.U)
      when(mag === 0.U) {
        r := h(15, 0)
      }.otherwise {
        val shifted = (h >> mag).asSInt
        val round = (h >> (mag - 1.U))(0) // LSB just below cut
        r := (shifted + round.asSInt)(15, 0)
      }
    }.otherwise {
      r := clipS16((h.pad(32) << shamt_mag).asSInt)
    }
    r
  })
  val psshar_result = psshar_lanes(1) ## psshar_lanes(0)

  // PSSHL.HS: saturating unsigned shift, 2 × 16-bit
  val psshl_lanes = VecInit((0 until 2).map { i =>
    val h = rs1(16 * i + 15, 16 * i) // unsigned 16-bit lane
    val r = Wire(UInt(16.W))
    when(shamt_neg) {
      when(shamt_mag >= 16.U) { r := 0.U }
        .otherwise { r := (h >> shamt_mag)(15, 0) }
    }.otherwise {
      r := clipU16(h << shamt_mag)
    }
    r
  })
  val psshl_result = psshl_lanes(1) ## psshl_lanes(0)

  // PSSHLR.HS: saturating unsigned shift with rounding
  val psshlr_lanes = VecInit((0 until 2).map { i =>
    val h = rs1(16 * i + 15, 16 * i)
    val r = Wire(UInt(16.W))
    when(shamt_neg) {
      val mag = shamt_mag.min(16.U)
      when(mag === 0.U) {
        r := h(15, 0)
      }.otherwise {
        val shifted = h >> mag
        val round = (h >> (mag - 1.U))(0)
        r := (shifted + round)(15, 0)
      }
    }.otherwise {
      r := clipU16(h << shamt_mag)
    }
    r
  })
  val psshlr_result = psshlr_lanes(1) ## psshlr_lanes(0)

  // PNCLIPI.B / PNCLIPRI.B
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

  // PNCLIPI.H / PNCLIPRI.H
  val pnclipi_h_lane0 = clipS16((rs1.asSInt >> shamt_i5).asSInt)
  val pnclipi_h_lane1 = clipS16((rs2.asSInt >> shamt_i5).asSInt)
  val pnclipi_h_result = pnclipi_h_lane1 ## pnclipi_h_lane0

  val pnclipri_h_lane0 = {
    val w = rs1.asSInt
    val shifted = (w >> shamt_i5).asSInt
    val round = Mux(shamt_i5 > 0.U, (w >> (shamt_i5 - 1.U))(0), 0.U)
    clipS16((shifted + round.asSInt).asSInt)
  }
  val pnclipri_h_lane1 = {
    val w = rs2.asSInt
    val shifted = (w >> shamt_i5).asSInt
    val round = Mux(shamt_i5 > 0.U, (w >> (shamt_i5 - 1.U))(0), 0.U)
    clipS16((shifted + round.asSInt).asSInt)
  }
  val pnclipri_h_result = pnclipri_h_lane1 ## pnclipri_h_lane0

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

  // PM2ADDA family: 2-cycle
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

  val c2_rs1 = io_reg.reg_read_data1
  val c2_lo1 = c2_rs1(15, 0); val c2_hi1 = c2_rs1(31, 16)
  val c2_lo2 = adda_rs2_saved(15, 0); val c2_hi2 = adda_rs2_saved(31, 16)

  val adda_dot_ss =
    (s16(c2_lo1) * s16(c2_lo2) + s16(c2_hi1) * s16(c2_hi2))(31, 0)
  val adda_dot_su =
    (s16(c2_lo1) * u16(c2_lo2) + s16(c2_hi1) * u16(c2_hi2))(31, 0)
  val adda_dot_uu =
    (u16(c2_lo1) * u16(c2_lo2) + u16(c2_hi1) * u16(c2_hi2))(31, 0)
  val adda_dot_sx =
    (s16(c2_lo1) * s16(c2_hi2) + s16(c2_hi1) * s16(c2_lo2))(31, 0)

  val adda_dot = MuxLookup(adda_type_reg.asUInt, adda_dot_ss)(
    Seq(
      RISCV_TYPE.pm2adda_h.asUInt -> adda_dot_ss,
      RISCV_TYPE.pm2addasu_h.asUInt -> adda_dot_su,
      RISCV_TYPE.pm2addau_h.asUInt -> adda_dot_uu,
      RISCV_TYPE.pm2adda_hx.asUInt -> adda_dot_sx
    )
  )
  val adda_result = (adda_rd_saved + adda_dot)(31, 0)

  // Dispatch
  when(io.valid) {
    when(adda_cycle2) {
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
      io_reg.reg_rs1 := io.instr(11, 7)
      io_reg.reg_rs2 := io.instr(24, 20)
      adda_rd_saved := io_reg.reg_read_data1
      adda_rs2_saved := io_reg.reg_read_data2
      adda_instr_reg := io.instr
      adda_type_reg := io.instr_type
      io.stall := STALL_REASON.EXECUTION_UNIT
      io_pc.pc_we := false.B
      adda_cycle2 := true.B

    }.otherwise {
      // Base register logic is handled statically above; defaults apply here safely
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
