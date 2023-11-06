using riscV;
using System.CommandLine;

class Program
{
    private const string imagePath = "../mini-rv32ima-files/DownloadedImage";
    private const int memoryOffset = unchecked((int)0x80000000);
    private const int offsetConst = 192;
    private const int ramAmount = 64 * 1024 * 1024;
    public static void Main(string[] args)
    {
        Option<bool> printStateOption = new(
            name: "--state",
            description: "Print the CPU state after ever cycle",
            getDefaultValue: () => false
        );
        printStateOption.AddAlias("-s");

        Option<int> maxCyclesOption = new(
            name: "--cycles",
            description: "The maximum number of instructions to read",
            getDefaultValue: () => -1
        );
        maxCyclesOption.AddAlias("-c");

        RootCommand rootCommand = new("Console implementation of a Risc-V CPU running Linux")
        {
            printStateOption,
            maxCyclesOption
        };

        rootCommand.SetHandler((printState, maxCycles) =>
        {
            Memory memory = new(ramAmount);

            // Load image
            using FileStream? image = File.OpenRead(imagePath);
            memory.LoadImage(image);

            // Load dtb
            int ptrDTB = ramAmount - DTB.Data.Length - offsetConst;
            memory.LoadDTB(ptrDTB, DTB.Data);

            MyCPU cpu = new(memory, ptrDTB + memoryOffset, printState, maxCycles);

            // EnterVirtualConsole();

            cpu.Start();
        }, printStateOption, maxCyclesOption);

        rootCommand.Invoke(args);
    }

    public static void OnProcessExit(object sender, EventArgs e)
    {
        // ExitVirtualConsole();
        // Console.WriteLine("We back");
    }

    class MyCPU : CPU
    {
        private long oldTime;

        public MyCPU(Memory memory, int reg11val, bool dumpState, int maxCycles) : base(memory, reg11val, dumpState, maxCycles)
        {
            oldTime = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
        }

        public override void HandleMemoryStore(int position, int data)
        {
            if (position == 0x10000000)
            {
                char sign = (char)data;
                Console.Write((char)data);
            }
        }

        public override int HandleMemoryLoad(int position)
        {
            // Emulating a 8250 / 16550 UART
            if (position == 0x10000005)
            {
                return Console.KeyAvailable ? 0x61 : 0x60;
            }
            else if (position == 0x10000000 && Console.KeyAvailable)
            {
                throw new NotImplementedException("Keyboard read not implemented 2");
            }

            return 0;
        }

        public override void HandleUnknownCSRWrite(int CSR, int value)
        {
            switch (CSR)
            {
                case 0x139:
                    Console.Write((char)value);
                    break;
                case 0x140:
                    break;
                default:
                    throw new NotImplementedException($"CSR write not seen before: {CSR:x}");
            }
        }

        public override int HandleUnknownCSRRead(int CSR)
        {
            switch (CSR)
            {
                case 0x139:
                    break;
                case 0x140:
                    return Console.KeyAvailable ? Console.ReadKey(true).KeyChar : -1;
                default:
                    throw new NotImplementedException($"CSR read not seen before: {CSR:x}");
            }

            return 0;
        }

        public override uint HandleTimeDiff()
        {
            long elapsedTime = (DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond) - oldTime;
            oldTime += elapsedTime;

            return Convert.ToUInt32(elapsedTime);
        }
    }

    public static void EnterVirtualConsole()
    {
        Console.Write("\x1b[?1049h");
        Console.Clear();
    }

    public static void ExitVirtualConsole()
    {
        Console.Write("\x1b[?1049l");
    }
}