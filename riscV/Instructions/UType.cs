namespace riscV.instructions;

class UType
{
    public int Opcode { get; private set; }
    public int Rd { get; private set; }
    public int Imm { get; private set; }

    public UType(int instruction)
    {
        Opcode = instruction & 0b1111111;
        Rd = (instruction >> 7) & 0b11111;
        Imm = (instruction >> 12) & 0b11111111111111111111;
    }
}