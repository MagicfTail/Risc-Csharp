namespace riscV.instructions;

class SType
{
    public int Opcode { get; private set; }
    public int Funct3 { get; private set; }
    public int Rs1 { get; private set; }
    public int Rs2 { get; private set; }
    public int Imm { get; private set; }

    public SType(int instruction)
    {
        Opcode = instruction & 0b1111111;
        Funct3 = (instruction >> 12) & 0b111;
        Rs1 = (instruction >> 15) & 0b11111;
        Rs2 = (instruction >> 20) & 0b11111;

        int Imm4_0 = (instruction >> 7) & 0b11111;
        int Imm11_5 = (instruction >> 20) & 0b111111100000;

        Imm = Imm11_5 | Imm4_0;
    }
}