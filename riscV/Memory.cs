using System.Text;

namespace riscV;

public class Memory
{
    private const int memoryOffset = unchecked((int)0x80000000);
    private readonly byte[] _RAM;
    private MemoryStream _stream;

    public Memory(int memorySizeBytes)
    {
        _RAM = new byte[memorySizeBytes];
        _stream = new(_RAM);
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

    public void LoadByteArray(byte[] data, int position)
    {
        _stream.Position = position;
        _stream.Write(data, 0, data.Length);
    }

    public void WriteByte(int position, byte data)
    {
        _stream.Position = position - memoryOffset;
        _stream.WriteByte(data);
    }

    public void WriteShort(int position, short data)
    {
        _stream.Position = position - memoryOffset;
        _stream.Write(BitConverter.GetBytes(data), 0, 2);
    }

    public void WriteInt(int position, int data)
    {
        _stream.Position = position - memoryOffset;
        _stream.Write(BitConverter.GetBytes(data), 0, 4);
    }

    public byte ReadByte(int position)
    {
        byte[] buffer = new byte[1];
        _stream.Position = position - memoryOffset;
        _stream.Read(buffer, 0, 1);

        return buffer[0];
    }

    public short ReadShort(int position)
    {
        byte[] buffer = new byte[2];
        _stream.Position = position - memoryOffset;
        _stream.Read(buffer, 0, 2);

        return BitConverter.ToInt16(buffer);
    }

    public int ReadInt(int position)
    {
        byte[] buffer = new byte[4];
        _stream.Position = position - memoryOffset;
        _stream.ReadExactly(buffer, 0, 4);
        return BitConverter.ToInt32(buffer);
    }
}
