namespace riscV.instructions;

class JType
{
    public int Opcode { get; private set; }
    public int Rd { get; private set; }
    public int Imm { get; private set; }

    public JType(int instruction)
    {
        Opcode = instruction & 0b1111111;
        Rd = (instruction >> 7) & 0b11111;

        var imm20 = (instruction >> 11) & 0b100000000000000000000;
        var imm10_1 = (instruction >> 20) & 0b11111111110;
        var imm11 = (instruction >> 9) & 0b100000000000;
        var imm19_12 = (instruction >> 0) & 0b11111111000000000000;

        Imm = imm20 | imm19_12 | imm11 | imm10_1;
    }
}