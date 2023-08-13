#ifndef _MACRO_H_
#define _MACRO_H_


// R-type
#define ADD_FUNC 0x20 // Done // Tested
#define ADDU_FUNC 0x21 // Done
#define AND_FUNC 0x24 // Done // Tested
#define NOR_FUNC 0x27 // Done
#define OR_FUNC 0x25 // Tested
#define SLL_FUNC 0x0 // Done // Shift Left Logical
#define SLLV_FUNC 0x4 // Done // Shift Left Logical Variable
#define SLT_FUNC 0x2A // Done // Set on Less Than
#define SLTU_FUNC 0x2B // Done // Set on Less Than Unsigned
#define SRA_FUNC 0x3 // Done // Shift Right Arithmetics
#define SRAV_FUNC 0x7 // Done // Shift Right Arithmetics Variable
#define SRL_FUNC 0x2 // Done // Shift Right Logical
#define SRLV_FUNC 0x6 // Done // Shift Right Logical Variable
#define SUB_FUNC 0x22 // Done
#define SUBU_FUNC 0x23 // Done
#define SYSCALL 0x0C // Done
#define XOR_FUNC 0x26 // Done 
#define JALR_FUNC 0x9
#define JR_FUNC 0x8
#define MULT_FUNC 0x18
#define MULTU_FUNC 0x19
#define DIV_FUNC 0x1A
#define DIVU_FUNC 0x1B
#define MFHI_FUNC 0x10
#define MFLO_FUNC 0x12
#define MTHI_FUNC 0x11
#define MTLO_FUNC 0x13

// ALU make up function
#define SHIFT_UPPER 255


// OPCODE
#define OPCODE_SPECIAL 0 
#define OPCODE_REGIMM  1
#define OPCODE_J       2 // done
#define OPCODE_JAL     3 // done
#define OPCODE_BEQ     4 // done
#define OPCODE_BNE     5 // done
#define OPCODE_BLEZ    6 
#define OPCODE_BGTZ    7
#define OPCODE_ADDI    8 // done
#define OPCODE_ADDIU   9 
#define OPCODE_SLTI   10
#define OPCODE_SLTIU  11
#define OPCODE_ANDI   12 // done
#define OPCODE_ORI    13 // done
#define OPCODE_XORI   14 // done
#define OPCODE_LUI    15 // done
#define OPCODE_LB     32
#define OPCODE_LH     33
#define OPCODE_LW     35 // done 
#define OPCODE_LBU    36
#define OPCODE_LHU    37
#define OPCODE_SB     40
#define OPCODE_SH     41
#define OPCODE_SW     43 // done


// REGIMM
#define REGIMM_BGEZ 0x1
#define REGIMM_BGEZAL 0x11
#define REGIMM_BLTZ 0x0
#define REGIMM_BLTZAL 0x10






#define ADD 0
#define SUB 1
#define SUBU 91
#define AND 2
#define OR 3
#define XOR 4
#define SLL 5
#define SRL 6
#define SRA 7
#define SLT 8
#define NOR  9
#define SLLV 10
#define SRLV 11
#define SRAV 12
#define MULT 13
#define MULTU 14
#define DIV 15
#define DIVU 16
#define MFHI 17
#define MFLO 18
#define MTHI 19
#define MTLO 20

#endif