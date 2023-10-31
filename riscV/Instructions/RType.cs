namespace riscV.instructions;

class RType
{
    public int Opcode { get; private set; }
    public int Rd { get; private set; }
    public int Funct3 { get; private set; }
    public int Rs1 { get; private set; }
    public int Rs2 { get; private set; }
    public int Funct7 { get; private set; }

    public RType(int instruction)
    {
        Opcode = (instruction) & 0b1111111;
        Rd = (instruction >> 7) & 0b11111;
        Funct3 = (instruction >> 12) & 0b111;
        Rs1 = (instruction >> 15) & 0b11111;
        Rs2 = (instruction >> 20) & 0b11111;
        Funct7 = (instruction >> 25) & 0b1111111;
    }
}