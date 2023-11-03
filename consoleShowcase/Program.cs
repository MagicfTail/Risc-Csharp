using riscV;

class Program
{
    private const string imagePath = "../mini-rv32ima-files/DownloadedImage";
    private const int memoryOffset = unchecked((int)0x80000000);
    private const int offsetConst = 192;
    private const int ramAmount = 64 * 1024 * 1024;
    public static void Main()
    {
        Memory memory = new(ramAmount);

        // Load image
        using FileStream? image = File.OpenRead(imagePath);
        memory.LoadImage(image);

        // Load dtb
        int ptrDTB = ramAmount - DTB.Data.Length - offsetConst;
        memory.LoadDTB(ptrDTB, DTB.Data);

        MyCPU cpu = new(memory, ptrDTB + memoryOffset);
        cpu.Start();
    }

    class MyCPU : CPU
    {
        public MyCPU(Memory memory, int reg11val) : base(memory, reg11val)
        { }

        public override void HandleMemoryStore(int position, int data)
        {
            if (position == 0x10000000)
            {
                Console.Write((char)data);
            }
        }

        public override int HandleMemoryLoad(int position)
        {
            // Emulating a 8250 / 16550 UART
            if (position == 0x10000005)
                throw new NotImplementedException("Keyboard read not implemented 1");
            else if (position == 0x10000000)// && IsKBHit())
                throw new NotImplementedException("Keyboard read not implemented 2");
            return 0;
        }
    }
}