package com.frc7153.Coprocessor;

public class IPAdress {
    private String address;
    private boolean error = false;

    private reportError(String message) {
        System.out.println(message);
        // TODO system-wide error/fault handler + reporter
        error = true;
    }

    private boolean checkIs8Bit(int... values) {
        for (int x = 0; x < values.length; x++) {
            if (x < 0 || x > 255) { return false; }
        }
        return true;
    }

    public IPAdress(int octet1, int octet2, int octet3, int octet4) {
        if (!checkIs8Bit(octet1, octet2, octet3, octet4)) {reportError("All IP address octets must be 8 bit (0 - 255) integers"); };
        address = String.format("%s.%s.%s.%s", octet1, octet2, octet3, octet4);
    }

    public IPAdress(String hostname) {
        address = hostname;
    }
}