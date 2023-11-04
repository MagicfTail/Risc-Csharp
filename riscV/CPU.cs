using System.Runtime.Intrinsics.X86;
using riscV.instructions;

namespace riscV;

public abstract class CPU
{
    private const int memoryOffset = unchecked((int)0x80000000);

    private const int mvendoridCSR = unchecked((int)0xff0ff0ff);
    private const int marchidCSR = 0x40401101;
    private const int mimpidCSR = 0x0;

    private readonly Memory _memory;

    private readonly int[] _registers = new int[32];

    // ---------- Debug options ----------
    private readonly bool _dumpState;
    private readonly int _maxCycles;
    private int cycle;
    // -----------------------------------

    private int _pc;

    private int LRAddress;
    private long mTime = 0;
    // private uint mTimeL = 0;
    // private uint mTimeH = 0;
    private long mTimeCmp = 0;
    // private uint mTimeCmpL = 0;
    // private uint mTimeCmpH = 0;
    private int globalTrap = 0;

    private bool WFI;
    private int privilege;

    private int mieCSR;
    private int mipCSR;
    private int mscratchCSR;
    private int mtvecCSR;
    private int pmpaddr0CSR;
    private int pmpcfg0CSR;
    private int mhartidCSR;
    private int mstatusCSR;
    private int mepcCSR;
    private int mtvalCSR;
    private int mcauseCSR;

    public CPU(Memory memory, int reg11Val, bool dumpState = false, int maxCycles = -1)
    {
        _pc = memoryOffset;
        _memory = memory;
        WriteRegister(11, reg11Val);

        _dumpState = dumpState;
        _maxCycles = maxCycles;
    }

    public abstract void HandleMemoryStore(int position, int data);

    public abstract int HandleMemoryLoad(int position);

    public abstract void HandleUnknownCSRWrite(int CSR, int value);

    public abstract int HandleUnknownCSRRead(int CSR);

    public abstract uint HandleTimeDiff();

    public void Start()
    {
        cycle = 0;
        while (_maxCycles < 0 || cycle < _maxCycles)
        {
            // if (cycle % 1024 == 0)
            // {
            //     UpdateTime(HandleTimeDiff());
            // }

            // if (cycle == 3248624)
            // {
            //     _memory.DumpMemory("../RAMDumpMine.tmp");
            //     Environment.Exit(0);
            // }


            int returnCode = Cycle();
            switch (returnCode)
            {
                case 0:
                    break;
                case 1:
                    UpdateTime(1);
                    break;
                default:
                    throw new NotImplementedException($"Return code not handled: {returnCode}");
            }
            UpdateTime(1);
            cycle++;
        }
    }

    public int Cycle()
    {
        int instructionReturnValue = 0;
        // Update every instruction, change later to regular updates

        if (mTime > mTimeCmp && mTimeCmp > 0)
        {
            WFI = false;
            mipCSR |= 1 << 7; //MTIP of MIP // https://stackoverflow.com/a/61916199/2926815  Fire interrupt.
        }
        else
        {
            mipCSR &= ~(1 << 7);
        }

        // If waiting for interrupt, don't do anything. This is where we should be sleeping
        if (WFI)
        {
            return 1;
        }

        if (((mipCSR & (1 << 7)) != 0) && ((mieCSR & (1 << 7)) != 0) && ((mstatusCSR & 0x8) != 0))
        {
            globalTrap = unchecked((int)0x80000007);
            _pc -= 4;
        }
        else
        {
            int instruction = ReadMemory(_pc, 32);
            instructionReturnValue = HandleInstruction32(instruction);
        }

        HandleTrap();

        return instructionReturnValue;
    }

    private int ReadCSR(int CSR)
    {
        return CSR switch
        {
            0x300 => mstatusCSR,
            0x304 => mieCSR,
            0x305 => mtvecCSR,
            0x340 => mscratchCSR,
            0x341 => mepcCSR,
            0x342 => mcauseCSR,
            0x343 => mtvalCSR,
            0x344 => mipCSR,
            0x3A0 => 0, //pmpcfg0CSR,
            0x3B0 => 0, //pmpaddr0CSR,
            0xf11 => 0, //mvendoridCSR,
            0xf12 => 0, //marchidCSR,
            0xf13 => 0, //mimpidCSR,
            0xF14 => 0, //mhartidCSR,
            _ => HandleUnknownCSRRead(CSR)
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
            case 0x341:
                mepcCSR = value;
                break;
            case 0x342:
                mcauseCSR = value;
                break;
            case 0x343:
                mtvalCSR = value;
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
            case 0xf11:
                break;
            case 0xf12:
                break;
            case 0xf13:
                break;
            case 0xF14:
                mhartidCSR = value;
                break;
            default:
                HandleUnknownCSRWrite(CSR, value);
                break;
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

    private int HandleInstruction32(int instruction)
    {
        int opcode = instruction & 0b1111111;

        if (_dumpState)
        {
            PrintStatus(instruction);
        }

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
                            switch ((unpacked.Imm >> 5) & 0b1111111)
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
                        case (0b010, 0b00001):
                            AMOSWAP_W(unpacked);
                            break;
                        case (0b010, 0b00010):
                            LR_W(unpacked);
                            break;
                        case (0b010, 0b00011):
                            SC_W(unpacked);
                            break;
                        case (0b010, 0b01000):
                            AMOOR_W(unpacked);
                            break;
                        case (0b010, 0b01100):
                            AMOAND_W(unpacked);
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
                        case (0b000, 0b0000001):    // RV32M Standard Extension
                            MUL(unpacked);
                            break;
                        case (0b000, 0b0100000):
                            SUB(unpacked);
                            break;
                        case (0b001, 0b0000000):
                            SLL(unpacked);
                            break;
                        case (0b001, 0b0000001):    // RV32M Standard Extension
                            MULH(unpacked);
                            break;
                        case (0b010, 0b0000000):
                            SLT(unpacked);
                            break;
                        case (0b011, 0b0000000):
                            SLTU(unpacked);
                            break;
                        case (0b011, 0b0000001):    // RV32M Standard Extension
                            MULHU(unpacked);
                            break;
                        case (0b100, 0b0000000):
                            XOR(unpacked);
                            break;
                        case (0b100, 0b0000001):    // RV32M Standard Extension
                            DIV(unpacked);
                            break;
                        case (0b101, 0b0000000):
                            SRL(unpacked);
                            break;
                        case (0b101, 0b0000001):    // RV32M Standard Extension
                            DIVU(unpacked);
                            break;
                        case (0b101, 0b0100000):
                            SRA(unpacked);
                            break;
                        case (0b110, 0b0000000):
                            OR(unpacked);
                            break;
                        case (0b110, 0b0000001):    // RV32M Standard Extension
                            REM(unpacked);
                            break;
                        case (0b111, 0b0000000):
                            AND(unpacked);
                            break;
                        case (0b111, 0b0000001):    // RV32M Standard Extension
                            REMU(unpacked);
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
                        case (0b000, 0b0000001):
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
                            throw new NotImplementedException($"Unhandled {ToBin(unpacked.Opcode)} funct: ({ToBin(unpacked.Funct3)}, {ToBin(unpacked.Imm)})");
                    }

                    break;
                }
            default:
                throw new NotImplementedException($"Unhandled opcode: {ToBin(opcode)}");
        }

        _pc += 4;
        return 0;
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
        WriteRegister(instruction.Rd, Sext(instruction.Imm << 12, 32));
    }

    private void AUIPC(UType instruction)
    {
        WriteRegister(instruction.Rd, _pc + (instruction.Imm << 12));
    }

    private void JAL(JType instruction)
    {
        WriteRegister(instruction.Rd, _pc + 4);
        int add = Sext(instruction.Imm, 21);
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
        if (ReadRegister(instruction.Rs1) >= ReadRegister(instruction.Rs2))
        {
            _pc += Sext(instruction.Imm, 13) - 4;
        }
    }

    private void BLTU(BType instruction)
    {
        if ((uint)ReadRegister(instruction.Rs1) < (uint)ReadRegister(instruction.Rs2))
        {
            _pc += Sext(instruction.Imm, 13) - 4;
        }
    }

    private void BGEU(BType instruction)
    {
        if ((uint)ReadRegister(instruction.Rs1) >= (uint)ReadRegister(instruction.Rs2))
        {
            _pc += Sext(instruction.Imm, 13) - 4;
        }
    }

    private void LB(IType instruction)
    {
        WriteRegister(instruction.Rd,
            Sext(ReadMemory(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12), 8) & 0b11111111, 8));
    }

    private void LH(IType instruction)
    {
        WriteRegister(instruction.Rd,
            Sext(ReadMemory(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12), 16) & 0b1111111111111111, 16));
    }

    private void LW(IType instruction)
    {
        WriteRegister(instruction.Rd, Sext(ReadMemory(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12), 32), 32));
    }

    private void LBU(IType instruction)
    {
        WriteRegister(instruction.Rd, ReadMemory(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12), 8) & 0b11111111);
    }

    private void LHU(IType instruction)
    {
        WriteRegister(instruction.Rd, ReadMemory(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12), 16) & 0b1111111111111111);
    }

    private void SB(SType instruction)
    {
        WriteMemory(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12), ReadRegister(instruction.Rs2), 8);
    }

    private void SH(SType instruction)
    {
        WriteMemory(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12), ReadRegister(instruction.Rs2), 16);
    }

    private void SW(SType instruction)
    {
        WriteMemory(ReadRegister(instruction.Rs1) + Sext(instruction.Imm, 12), ReadRegister(instruction.Rs2), 32);
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
        WriteRegister(instruction.Rd, (uint)ReadRegister(instruction.Rs1) < (uint)Sext(instruction.Imm, 12) ? 1 : 0);
    }

    private void XORI(IType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) ^ Sext(instruction.Imm, 12));
    }

    private void ORI(IType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) | Sext(instruction.Imm, 12));
    }

    private void ANDI(IType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) & Sext(instruction.Imm, 12));
    }

    private void SLLI(IType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) << (instruction.Imm & 0b11111));
    }

    private void SRLI(IType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) >>> (instruction.Imm & 0b11111));
    }

    private void SRAI(IType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) >> (instruction.Imm & 0b11111));
    }

    private void ADD(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) + ReadRegister(instruction.Rs2));
    }

    private void SUB(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) - ReadRegister(instruction.Rs2));
    }

    private void SLL(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) << (ReadRegister(instruction.Rs2) & 0b11111));
    }

    private void SLT(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) < ReadRegister(instruction.Rs2) ? ReadRegister(instruction.Rs1) : 0);
    }

    private void SLTU(RType instruction)
    {
        WriteRegister(instruction.Rd, (uint)ReadRegister(instruction.Rs1) < (uint)ReadRegister(instruction.Rs2) ? 1 : 0);
    }

    private void XOR(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) ^ ReadRegister(instruction.Rs2));
    }

    private void SRL(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) >>> (ReadRegister(instruction.Rs2) & 0b11111));
    }

    private void SRA(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) >> (ReadRegister(instruction.Rs2) & 0b11111));
    }

    private void OR(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) | ReadRegister(instruction.Rs2));
    }

    private void AND(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) & ReadRegister(instruction.Rs2));
    }

    private void FENCE(IType instruction)
    {
        // fence isn't required for this implementation
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
        int t = ReadCSR(instruction.Imm);
        WriteCSR(instruction.Imm, t | instruction.Rs1);
        WriteRegister(instruction.Rd, t);
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

    private void MULH(RType instruction)
    {
        long data = (long)ReadRegister(instruction.Rs1) * (long)ReadRegister(instruction.Rs2);
        WriteRegister(instruction.Rd, (int)(data >>> 32));
    }

    private void MULHU(RType instruction)
    {
        ulong data = (ulong)(uint)ReadRegister(instruction.Rs1) * (uint)ReadRegister(instruction.Rs2);
        WriteRegister(instruction.Rd, (int)(data >>> 32));
    }

    private void DIV(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) / ReadRegister(instruction.Rs2));
    }

    private void DIVU(RType instruction)
    {
        WriteRegister(instruction.Rd, (int)((uint)ReadRegister(instruction.Rs1) / (uint)ReadRegister(instruction.Rs2)));
    }

    private void REM(RType instruction)
    {
        WriteRegister(instruction.Rd, ReadRegister(instruction.Rs1) % ReadRegister(instruction.Rs2));
    }

    private void REMU(RType instruction)
    {
        WriteRegister(instruction.Rd, (int)((uint)ReadRegister(instruction.Rs1) % (uint)ReadRegister(instruction.Rs2)));
    }

    // RV32A Standard Extension

    private void LR_W(RType instruction)
    {
        int address = ReadRegister(instruction.Rs1);
        WriteRegister(instruction.Rd, ReadMemory(address, 32));
        LRAddress = address;
    }

    private void SC_W(RType instruction)
    {
        if (LRAddress == ReadRegister(instruction.Rs1))
        {
            WriteMemory(ReadRegister(instruction.Rs1), ReadRegister(instruction.Rs2), 32);
            WriteRegister(instruction.Rd, 0);
        }
        else
        {
            WriteRegister(instruction.Rd, 1);
        }
    }

    private void AMOADD_W(RType instruction)
    {
        int data = ReadMemory(ReadRegister(instruction.Rs1), 32);
        WriteMemory(ReadRegister(instruction.Rs1), data + ReadRegister(instruction.Rs2), 32);
        WriteRegister(instruction.Rd, data);
    }

    private void AMOSWAP_W(RType instruction)
    {
        int data = ReadMemory(ReadRegister(instruction.Rs1), 32);
        WriteRegister(instruction.Rd, data);
        WriteRegister(instruction.Rs1, ReadRegister(instruction.Rs2));
        WriteRegister(instruction.Rs2, data);
    }

    private void AMOOR_W(RType instruction)
    {
        int data = ReadMemory(ReadRegister(instruction.Rs1), 32);
        WriteMemory(ReadRegister(instruction.Rs1), data | ReadRegister(instruction.Rs2), 32);
        WriteRegister(instruction.Rd, data);
    }

    private void AMOAND_W(RType instruction)
    {
        int data = ReadMemory(ReadRegister(instruction.Rs1), 32);
        WriteMemory(ReadRegister(instruction.Rs1), data & ReadRegister(instruction.Rs2), 32);
        WriteRegister(instruction.Rd, data);
    }

    public static int Sext(int value, int length)
    {
        int checkMask = (int)Math.Pow(2, length - 1);

        if ((value & checkMask) == 0)
        {
            return value;
        }

        int signMask = (int)Math.Pow(2, length) - 1;

        return value | ~signMask;
    }

    private void WriteMemory(int position, int data, int len)
    {
        int offsetPosition = position - memoryOffset;

        if ((uint)offsetPosition >= _memory.Length - 3)
        {
            if ((uint)position >= 0x10000000 && (uint)position < 0x12000000)
            {
                if (position == 0x11004004) // CLNT
                {
                    mTimeCmp = ((long)data << 32) | (mTimeCmp & 0xffffffff);
                    return;
                }
                else if (position == 0x11004000) // CLNT
                {
                    mTimeCmp = (data & 0xffffffff) | (mTimeCmp & unchecked((long)0xffffffff00000000));
                    return;
                }
                else if (position == 0x11100000) //SYSCON (reboot, power off, etc.)
                {
                    throw new NotImplementedException($"Unimplemented Write Position SYSCON: {position}");
                    //return;
                }
                else
                {
                    HandleMemoryStore(position, data);
                    return;
                }
            }
            else
            {
                throw new InvalidDataException("Store access fault.");
            }
        }

        switch (len)
        {
            case 8:
                _memory.WriteByte(offsetPosition, (byte)(data & 0b11111111));
                break;
            case 16:
                _memory.WriteHalf(offsetPosition, (short)(data & 0b1111111111111111));
                break;
            case 32:
                _memory.WriteWord(offsetPosition, data);
                break;
            default:
                throw new InvalidDataException($"Invalid write length: {len}");
        }
    }

    public int ReadMemory(int position, int len)
    {
        int offsetPosition = position - memoryOffset;

        if ((uint)offsetPosition >= _memory.Length - 3)
        {
            if ((uint)position >= 0x10000000 && (uint)position < 0x12000000)  // UART, CLNT
            {
                if (position == 0x1100bffc) // https://chromitem-soc.readthedocs.io/en/latest/clint.html
                {
                    return (int)(mTime >>> 32);
                }
                else if (position == 0x1100bff8)
                {
                    return (int)(mTime & 0xffffffff);
                }
                else
                {
                    return HandleMemoryLoad(position);
                }
            }
            else
            {
                throw new InvalidDataException("Read access fault.");
            }
        }

        switch (len)
        {
            case 8:
                return _memory.ReadByte(offsetPosition) & 0b11111111;
            case 16:
                return _memory.ReadHalf(offsetPosition) & 0b1111111111111111;
            case 32:
                return _memory.ReadWord(offsetPosition);
            default:
                throw new InvalidDataException($"Invalid read length: {len}");
        }
    }

    private void UpdateTime(long difference)
    {
        mTime += difference;
    }

    private void HandleTrap()
    {
        if (globalTrap == 0)
        {
            return;
        }

        if ((globalTrap & 0x80000000) != 0) // Interrupt, not a trap
        {
            mcauseCSR = globalTrap;
            mtvalCSR = 0;
            _pc += 4;
        }
        else
        {
            mcauseCSR = globalTrap - 1;
            throw new NotImplementedException();
        }
        mepcCSR = _pc;
        mstatusCSR = ((mstatusCSR & 0x08) << 4) | ((privilege & 3) << 11);
        _pc = mtvecCSR - 4;
        // Set flags or smt;

        privilege = 3;

        globalTrap = 0;
        _pc += 4;
    }

    private static string ToBin(int value)
    {
        return Convert.ToString(value, 2);
    }

    private static string ToHex(int value)
    {
        return Convert.ToString(value, 16);
    }

    private void PrintStatus(int instruction)
    {
        Console.WriteLine($"x:{mstatusCSR:x8} a:{(int)(mTime >>> 32):x8} b:{Math.Max((int)((mTime & 0xffffffff) - 1), 0):x8} c:{(int)(mTimeCmp >>> 32):x8} d:{(int)(mTimeCmp & 0xffffffff):x8} e:{mscratchCSR:x8} f:{mtvecCSR:x8} g:{mieCSR:x8} h:{mipCSR:x8} i:{mepcCSR:x8} j:{mtvalCSR:x8} k:{mcauseCSR:x8}");

        Console.WriteLine("PC: " + $"""{_pc:X8} [0x{instruction:X8}] """.ToLower());

        Console.Write("Z" + $":{_registers[0]:X8} ra:{_registers[1]:X8} sp:{_registers[2]:X8} gp:{_registers[3]:X8} tp:{_registers[4]:X8} t0:{_registers[5]:X8} t1:{_registers[6]:X8} t2:{_registers[7]:X8} s0:{_registers[8]:X8} s1:{_registers[9]:X8} a0:{_registers[10]:X8} a1:{_registers[11]:X8} a2:{_registers[12]:X8} a3:{_registers[13]:X8} a4:{_registers[14]:X8} a5:{_registers[15]:X8} ".ToLower());
        Console.WriteLine($"a6:{_registers[16]:X8} a7:{_registers[17]:X8} s2:{_registers[18]:X8} s3:{_registers[19]:X8} s4:{_registers[20]:X8} s5:{_registers[21]:X8} s6:{_registers[22]:X8} s7:{_registers[23]:X8} s8:{_registers[24]:X8} s9:{_registers[25]:X8} s10:{_registers[26]:X8} s11:{_registers[27]:X8} t3:{_registers[28]:X8} t4:{_registers[29]:X8} t5:{_registers[30]:X8} t6:{_registers[31]:X8}".ToLower());
    }
}