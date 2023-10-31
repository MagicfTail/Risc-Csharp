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
        memory.LoadByteArray(DTB.Data, ptrDTB);

        CPU cpu = new(memory, ptrDTB + memoryOffset);

        cpu.Start();

    }
}