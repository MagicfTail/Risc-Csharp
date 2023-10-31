namespace riscV.instructions;

class BType
{
    public int Opcode { get; private set; }
    public int Rd { get; private set; }
    public int Funct3 { get; private set; }
    public int Rs1 { get; private set; }
    public int Rs2 { get; private set; }
    public int Imm { get; private set; }

    public BType(int instruction)
    {
        Opcode = instruction & 0b1111111;
        Funct3 = (instruction >> 12) & 0b111;
        Rs1 = (instruction >> 15) & 0b11111;
        Rs2 = (instruction >> 20) & 0b11111;
        int Imm11 = (instruction << 4) & 0b100000000000;
        int Imm4_1 = (instruction >> 7) & 0b11110;
        int Imm10_5 = (instruction >> 20) & 0b11111100000;
        int Imm12 = (instruction >> 19) & 0b1000000000000;

        Imm = Imm12 | Imm11 | Imm10_5 | Imm4_1;
    }
}