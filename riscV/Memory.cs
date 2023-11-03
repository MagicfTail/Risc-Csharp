using System.Text;

namespace riscV;

public class Memory
{
    private const int memoryOffset = unchecked((int)0x80000000);
    private readonly byte[] _RAM;
    private MemoryStream _stream;

    public int Length => _RAM.Length;

    public Memory(int memorySizeBytes)
    {
        _RAM = new byte[memorySizeBytes];
        _stream = new(_RAM);
    }

    // _memory.DumpMemory("../RAMDumpMine.tmp");
    // Environment.Exit(0);
    public void DumpMemory(string fileName)
    {
        using FileStream? file = File.Create(fileName);
        StringBuilder hex = new StringBuilder((int)(_stream.Length * 2));
        int c = 0;
        foreach (byte b in _RAM)
        {
            c += 1;
            hex.AppendFormat("{0:x2}", b);

            if (c % 8 == 0)
            {
                hex.Append('\n');
            }
        }
        file.Write(Encoding.UTF8.GetBytes(hex.ToString()));
    }

    public void LoadImage(FileStream? image)
    {
        if (image == null)
        {
            throw new IOException("Failed to read/load image");
        }

        _stream.Position = 0;
        image.CopyTo(_stream);
    }

    public void LoadDTB(int position, byte[] data)
    {
        _stream.Position = position;
        _stream.Write(data, 0, data.Length);

        // Update system ram size in DTB (but if and only if we're using the default DTB)
        // Warning - this will need to be updated if the skeleton DTB is ever modified.
        // See https://github.com/cnlohr/mini-rv32ima
        byte[] buffer = new byte[4];
        _stream.Position = position + 0x13c;
        _stream.Read(buffer, 0, 4);
        if (BitConverter.ToInt32(buffer) == 0x00c0ff03)
        {
            int validRAM = position;
            int data2 = (validRAM >> 24) | (((validRAM >> 16) & 0xff) << 8) | (((validRAM >> 8) & 0xff) << 16) | ((validRAM & 0xff) << 24);
            _stream.Position = position + 0x13c;
            _stream.Write(BitConverter.GetBytes(data2), 0, 4);
        }
    }

    public void WriteByte(int position, byte data)
    {
        _stream.Position = position;
        _stream.Write(new byte[1] { data }, 0, 1);
    }

    public void WriteHalf(int position, short data)
    {
        _stream.Position = position;
        _stream.Write(BitConverter.GetBytes(data), 0, 2);
    }

    public void WriteWord(int position, int data)
    {
        _stream.Position = position;
        _stream.Write(BitConverter.GetBytes(data), 0, 4);
    }

    public int ReadByte(int position)
    {
        byte[] buffer = new byte[1];
        _stream.Position = position;
        _stream.Read(buffer, 0, 1);

        return buffer[0];
    }

    public int ReadHalf(int position)
    {
        byte[] buffer = new byte[2];
        _stream.Position = position;
        _stream.Read(buffer, 0, 2);

        return BitConverter.ToInt16(buffer);
    }

    public int ReadWord(int position)
    {
        byte[] buffer = new byte[4];
        _stream.Position = position;
        _stream.ReadExactly(buffer, 0, 4);
        return BitConverter.ToInt32(buffer);
    }
}
