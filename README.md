# Risc-C#

A Risc-V emulator written in C#, capable of running a stripped down version of Linux. Heavily "inspired" by [mini-rv32ima](https://github.com/cnlohr/mini-rv32ima).

The implementation should be very human readable, with a streamlined flow from start to end of reading an instruction.

## Get it running

To see it in action, pass the console showcase the image stored in mini-rva32ima-files and login as root.

`dotnet run --project consoleShowcase -- ./mini-rv32ima-files/DownloadedImage`

(Or compile it yourself and run it however you want)

## Why

To get a better understanding of Risc-V, as well as the inner workings of an operating system.