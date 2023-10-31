using System.Runtime.Intrinsics.X86;
using riscV.instructions;

namespace riscV;

public class CPU
{
    private const int memoryOffset = unchecked((int)0x80000000);

    private readonly Memory _memory;

    private readonly int[] _registers = new int[32];

    private int _pc;
    private int mieCSR;
    private int mipCSR;
    private int mscratchCSR;
    private int mtvecCSR;
    private int pmpaddr0CSR;
    private int pmpcfg0CSR;
    private int mhartidCSR;
    private int mstatusCSR;

    public CPU(Memory memory, int reg11Val)
    {
        _pc = memoryOffset;
        _memory = memory;
        WriteRegister(11, reg11Val);
    }

    public void Start()
    {
        while (true)
        {
            Cycle();
        }
    }

    public void Cycle()
    {
        int instruction = _memory.ReadInt(_pc);
        HandleInstruction32(instruction);
        _pc += 4;
    }

    private int ReadCSR(int CSR)
    {
        return CSR switch
        {
            0x300 => mstatusCSR,
            0x304 => mieCSR,
            0x305 => mtvecCSR,
            0x340 => mscratchCSR,
            0x344 => mipCSR,
            0x3A0 => pmpcfg0CSR,
            0x3B0 => pmpaddr0CSR,
            0xF14 => mhartidCSR,
            _ => throw new InvalidOperationException($"Tried Reading unimplemented CSR register: {Convert.ToString(CSR, 16)}"),
        };
    }

    private void WriteCSR(int CSR, int value)
    {
        switch (CSR)
        {
            case 0x300:
                mstatusCSR = value;
                break;
            case 0x304:
                mieCSR = value;
                break;
            case 0x305:
                mtvecCSR = value;
                break;
            case 0x340:
                mscratchCSR = value;
                break;
            case 0x344:
                mipCSR = value;
                break;
            case 0x3A0:
                pmpcfg0CSR = value;
                break;
            case 0x3B0:
                pmpaddr0CSR = value;
                break;
            case 0xF14:
                mhartidCSR = value;
                break;
            default:
                throw new InvalidOperationException($"Tried writing unimplemented CSR register: {Convert.ToString(CSR, 16)}");
        }
    }

    private int ReadRegister(int register)
    {
        return _registers[register];
    }

    private void WriteRegister(int register, int value)
    {
        if (register == 0)
        {
            // throw new InvalidOperationException($"Tried writing register 0");
            return;
        }

        _registers[register] = value;
    }

    private void HandleInstruction32(int instruction)
    {
        int opcode = instruction & 0b1111111;

        Console.WriteLine("PC: " + $"""{_pc:X8} [0x{instruction:X8}] """.ToLower());
        Console.Write("Z" + $":{_registers[0]:X8} ra:{_registers[1]:X8} sp:{_registers[2]:X8} gp:{_registers[3]:X8} tp:{_registers[4]:X8} t0:{_registers[5]:X8} t1:{_registers[6]:X8} t2:{_registers[7]:X8} s0:{_registers[8]:X8} s1:{_registers[9]:X8} a0:{_registers[10]:X8} a1:{_registers[11]:X8} a2:{_registers[12]:X8} a3:{_registers[13]:X8} a4:{_registers[14]:X8} a5:{_registers[15]:X8} ".ToLower());
        Console.WriteLine($"a6:{_registers[16]:X8} a7:{_registers[17]:X8} s2:{_registers[18]:X8} s3:{_registers[19]:X8} s4:{_registers[20]:X8} s5:{_registers[21]:X8} s6:{_registers[22]:X8} s7:{_registers[23]:X8} s8:{_registers[24]:X8} s9:{_registers[25]:X8} s10:{_registers[26]:X8} s11:{_registers[27]:X8} t3:{_registers[28]:X8} t4:{_registers[29]:X8} t5:{_registers[30]:X8} t6:{_registers[31]:X8}".ToLower());

        switch (opcode)
        {
            case 0b0000011: // 0x03
                {
                    var unpacked = ITypeOpcode(instruction);

                    switch (unpacked.Funct3)
                    {
                        case 0b000:
                            LB(unpacked);
                            break;
                        case 0b001:
                            LH(unpacked);
                            break;
                        case 0b010:
                            LW(unpacked);
                            break;
                        case 0b100:
                            LBU(unpacked);
                            break;
                        case 0b101:
                            LHU(unpacked);
                            break;
                        default:
                            throw new NotImplementedException($"Unhandled {ToBin(unpacked.Opcode)} funct: ({ToBin(unpacked.Funct3)})");
                    }

                    break;
                }
            case 0b0001111: // 0x0F
                {
                    var unpacked = ITypeOpcode(instruction);

                    switch (unpacked.Funct3)
                    {
                        case 0b000:
                            FENCE(unpacked);
                            break;
                        case 0b001:
                            FENCE_I(unpacked);
                            break;
                        default:
                            throw new NotImplementedException($"Unhandled {ToBin(unpacked.Opcode)} funct: ({ToBin(unpacked.Funct3)})");
                    }

                    break;
                }
            case 0b0010011: // 0x13
                {
                    var unpacked = ITypeOpcode(instruction);

                    switch (unpacked.Funct3)
                    {
                        case 0b000:
                            ADDI(unpacked);
                            break;
                        case 0b001:
                            SLLI(unpacked);     // RV64I Base Instruction Set
                            break;
                        case 0b010:
                            SLTI(unpacked);
                            break;
                        case 0b011:
                            SLTIU(unpacked);
                            break;
                        case 0b100:
                            XORI(unpacked);
                            break;
                        case 0b101:
                            switch ((unpacked.Imm >> 5) & 0b11111)
                            {
                                case 0b0000000:
                                    SRLI(unpacked);
                                    break;
                                case 0b0100000:
                                    SRAI(unpacked);
                                    break;
                                default:
                                    throw new NotImplementedException($"Unhandled {ToBin(unpacked.Opcode)} funct: ({ToBin(unpacked.Funct3)}, {ToBin(unpacked.Imm >> 5)})");
                            }
                            break;
                        case 0b110:
                            ORI(unpacked);
                            break;
                        case 0b111:
                            ANDI(unpacked);
                            break;
                        default:
                            throw new NotImplementedException($"Unhandled {ToBin(unpacked.Opcode)} funct: ({ToBin(unpacked.Funct3)})");
                    }

                    break;
                }
            case 0b0010111: // 0x17
                {
                    var unpacked = UTypeOpcode(instruction);
                    AUIPC(unpacked);

                    break;
                }
            case 0b0100011: // 0x23
                {
                    var unpacked = STypeOpcode(instruction);

                    switch (unpacked.Funct3)
                    {
                        case 0b000:
                            SB(unpacked);
                            break;
                        case 0b001:
                            SH(unpacked);
                            break;
                        case 0b010:
                            SW(unpacked);
                            break;
                        default:
                            throw new NotImplementedException($"Unhandled {ToBin(unpacked.Opcode)} funct: ({ToBin(unpacked.Funct3)})");
                    }

                    break;
                }
            case 0b0101111: // 0x2F RV32A Standard Extension
                {
                    var unpacked = RTypeOpcode(instruction);

                    switch ((unpacked.Funct3, (unpacked.Funct7 >> 2) & 0b11111))
                    {
                        case (0b010, 0b00000):
                            AMOADD_W(unpacked);
                            break;
                        case (0b010, 0b01000):
                            AMOOR_W(unpacked);
                            break;
                        default:
                            throw new NotImplementedException($"Unhandled {ToBin(unpacked.Opcode)} funct: ({ToBin(unpacked.Funct3)}, {ToBin(unpacked.Funct7 >> 2)})");
                    }

                    break;
                }
            case 0b0110011: // 0x33
                {
                    var unpacked = RTypeOpcode(instruction);

                    switch ((unpacked.Funct3, unpacked.Funct7))
                    {
                        case (0b000, 0b0000000):
                            ADD(unpacked);
                            break;
                        case (0b000, 0b0100000):
                            SUB(unpacked);
                            break;
                        case (0b001, 0b0000000):
                            SLL(unpacked);
                            break;
                        case (0b010, 0b0000000):
                            SLT(unpacked);
                            break;
                        case (0b011, 0b0000000):
                            SLTU(unpacked);
                            break;
                        case (0b100, 0b0000000):
                            XOR(unpacked);
                            break;
                        case (0b100, 0b0000001):    // RV32M Standard Extension
                            MUL(unpacked);
                            break;
                        case (0b101, 0b0000000):
                            SRL(unpacked);
                            break;
                        case (0b101, 0b0100000):
                            SRA(unpacked);
                            break;
                        case (0b110, 0b0000000):
                            OR(unpacked);
                            break;
                        case (0b111, 0b0000000):
                            AND(unpacked);
                            break;
                        default:
                            throw new NotImplementedException($"Unhandled {ToBin(unpacked.Opcode)} funct: ({ToBin(unpacked.Funct3)}, {ToBin(unpacked.Funct7)})");
                    }

                    break;
                }
            case 0b0110111: // 0x37
                {
                    var unpacked = UTypeOpcode(instruction);
                    LUI(unpacked);

                    break;
                }
            case 0b1100011: // 0x63
                {
                    var unpacked = BTypeOpcode(instruction);

                    switch (unpacked.Funct3)
                    {
                        case 0b000:
                            BEQ(unpacked);
                            break;
                        case 0b001:
                            BNE(unpacked);
                            break;
                        case 0b100:
                            BLT(unpacked);
                            break;
                        case 0b101:
                            BGE(unpacked);
                            break;
                        case 0b110:
                            BLTU(unpacked);
                            break;
                        case 0b111:
                            BGEU(unpacked);
                            break;
                        default:
                            throw new NotImplementedException($"Unhandled {ToBin(unpacked.Opcode)} funct: ({ToBin(unpacked.Funct3)})");
                    }
                    break;
                }
            case 0b1100111: // 0x67
                {
                    var unpacked = ITypeOpcode(instruction);
                    JALR(unpacked);

                    break;
                }
            case 0b1101111: // 0x6F
                {
                    var unpacked = JTypeOpcode(instruction);
                    JAL(unpacked);

                    break;
                }
            case 0b1110011: // 0x73
                {
                    var unpacked = ITypeOpcode(instruction);

                    switch ((unpacked.Funct3, unpacked.Imm))
                    {
                        case (0b000, 0b0000000):
                            ECALL(unpacked);
                            break;
                        case (0b000, 0b0100000):
                            EBREAK(unpacked);
                            break;
                        case (0b001, _):
                            CSRRW(unpacked);
                            break;
                        case (0b010, _):
                            CSRRS(unpacked);
                            break;
                        case (0b011, _):
                            CSRRC(unpacked);
                            break;
                        case (0b101, _):
                            CSRRWI(unpacked);
                            break;
                        case (0b110, _):
                            CSRRSI(unpacked);
                            break;
                        case (0b111, _):
                            CSRRCI(unpacked);
                            break;
                        default:
                            throw new NotImplementedException($"Unhandled {ToBin(unpacked.Opcode)} funct: ({ToBin(unpacked.Funct3)})");
                    }

                    break;
                }
            default:
                throw new NotImplementedException($"Unhandled opcode: {ToBin(opcode)}");
        }
    }

    private RType RTypeOpcode(int instruction)
    {
        return new(instruction);
    }

    private IType ITypeOpcode(int instruction)
    {
        return new(instruction);
    }

    private SType STypeOpcode(int instruction)
    {
        return new(instruction);
    }

    private BType BTypeOpcode(int instruction)
    {
        return new(instruction);
    }

    private UType UTypeOpcode(int instruction)
    {
        return new(instruction);
    }

    private JType JTypeOpcode(int instruction)
    {
        return new(instruction);
    }

    private void LUI(UType instruction)
    {
        WriteRegister(instruction.Rd, instruction.Imm << 12);
    }

    private void AUIPC(UType instruction)
    {
        WriteRegister(instruction.Rd, _pc + (instruction.Imm << 12));
    }

    private void JAL(JType instruction)
    {
        WriteRegister(instruction.Rd, _pc + 4);
        _pc += Sext(instruction.Imm, 21) - 4;
    }

    private void JALR(IType instruction)
    {
        int t = _pc + 4;
        _pc = ((ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12)) & ~1) - 4;
        WriteRegister(instruction.Rd, t);
    }

    private void BEQ(BType instruction)
    {
        if (ReadRegister(instruction.Rs1) == ReadRegister(instruction.Rs2))
        {
            _pc += Sext(instruction.Imm, 13) - 4;
        }
    }

    private void BNE(BType instruction)
    {
        if (ReadRegister(instruction.Rs1) != ReadRegister(instruction.Rs2))
        {
            _pc += Sext(instruction.Imm, 13) - 4;
        }
    }

    private void BLT(BType instruction)
    {
        if (ReadRegister(instruction.Rs1) < ReadRegister(instruction.Rs2))
        {
            _pc += Sext(instruction.Imm, 13) - 4;
        }
    }

    private void BGE(BType instruction)
    {
        if (ReadRegister(instruction.Rs1) >= ReadRegister(instruction.Rs2)) _pc += Sext(instruction.Imm, 13) - 4;
    }

    private void BLTU(BType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void BGEU(BType instruction)
    {
        if ((uint)ReadRegister(instruction.Rs1) >= (uint)ReadRegister(instruction.Rs1))
        {
            _pc += Sext(instruction.Imm, 13) - 4;
        }
    }

    private void LB(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void LH(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void LW(IType instruction)
    {
        WriteRegister(instruction.Rd, _memory.ReadInt(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12)));
    }

    private void LBU(IType instruction)
    {
        WriteRegister(instruction.Rd, _memory.ReadByte(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12)) & 0b11111111);
    }

    private void LHU(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void SB(SType instruction)
    {
        byte data = (byte)(ReadRegister(instruction.Rs2) & 0b11111111);
        _memory.WriteByte(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12), data);
    }

    private void SH(SType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void SW(SType instruction)
    {
        _memory.WriteInt(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12), ReadRegister(instruction.Rs2));
    }

    private void ADDI(IType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12));
    }

    private void SLTI(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void SLTIU(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void XORI(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void ORI(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void ANDI(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void SLLI(IType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) << (instruction.Imm & 0b11111));
    }

    private void SRLI(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void SRAI(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void ADD(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) + ReadRegister(instruction.Rs2));
    }

    private void SUB(RType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void SLL(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) << (ReadRegister(instruction.Rs2) & 0b11111));
    }

    private void SLT(RType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void SLTU(RType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void XOR(RType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void SRL(RType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void SRA(RType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void OR(RType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void AND(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) & ReadRegister(instruction.Rs2));
    }

    private void FENCE(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void FENCE_I(IType instruction)
    {
        // fence.i isn't required for this implementation
    }

    private void ECALL(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void EBREAK(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void CSRRW(IType instruction)
    {
        int t = ReadCSR(instruction.Imm);
        WriteCSR(instruction.Imm, ReadRegister(instruction.Rs1));
        WriteRegister(instruction.Rd, t);
    }

    private void CSRRS(IType instruction)
    {
        int t = ReadCSR(instruction.Imm);
        WriteCSR(instruction.Imm, t | ReadRegister(instruction.Rs1));
        WriteRegister(instruction.Rd, t);
    }

    private void CSRRC(IType instruction)
    {
        int t = ReadCSR(instruction.Imm);
        WriteCSR(instruction.Imm, t & (~ReadRegister(instruction.Rs1)));
        WriteRegister(instruction.Rd, t);
    }

    private void CSRRWI(IType instruction)
    {
        WriteRegister(instruction.Rd, ReadCSR(instruction.Imm));
        WriteCSR(instruction.Imm, instruction.Rs1);
    }

    private void CSRRSI(IType instruction)
    {
        throw new NotImplementedException($"Unimplemented instruction: {System.Reflection.MethodBase.GetCurrentMethod()?.Name}");
    }

    private void CSRRCI(IType instruction)
    {
        int t = ReadCSR(instruction.Imm);
        WriteCSR(instruction.Imm, t & ~instruction.Rs1);
        WriteRegister(instruction.Rd, t);
    }

    // RV32M Standard Extension

    private void MUL(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) * ReadRegister(instruction.Rs2));
    }

    // RV32A Standard Extension

    private void AMOADD_W(RType instruction)
    {
        int data = _memory.ReadInt(ReadRegister(instruction.Rs1));
        WriteRegister(instruction.Rd, data);
        _memory.WriteInt(ReadRegister(instruction.Rs1), data + ReadRegister(instruction.Rs2));
    }

    private void AMOOR_W(RType instruction)
    {
        int data = _memory.ReadInt(ReadRegister(instruction.Rs1));
        WriteRegister(instruction.Rd, data);
        _memory.WriteInt(ReadRegister(instruction.Rs1), data | ReadRegister(instruction.Rs2));
    }

    private static int Sext(int value, int length)
    {
        int checkMask = (int)Math.Pow(2, length - 1);

        if ((value & checkMask) == 0)
        {
            return value;
        }

        int signMask = (int)Math.Pow(2, length) - 1;

        return value | ~signMask;
    }

    private static string ToBin(int value)
    {
        return Convert.ToString(value, 2);
    }

    private static string ToHex(int value)
    {
        return Convert.ToString(value, 16);
    }
}