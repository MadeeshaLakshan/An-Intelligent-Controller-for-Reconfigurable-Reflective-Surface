function uart_send_16bit_word(data_word, port_name)
    if nargin < 2
        port_name = 'COM5';
    end
    if nargin < 1
        data_word = uint16(hex2dec('ABCD')); % default 16-bit test value
    end

    data_word = bitand(data_word, 0xFFFF);  % Ensure 16-bit

    % Open serial port
    s = serialport(port_name, 115200);
    configureTerminator(s, "LF");
    s.DataBits = 8;
    s.Parity = "none";
    s.StopBits = 1;
    s.FlowControl = "none";

    try
        % Split into two bytes (little-endian: LSB first, then MSB)
        byte1 = bitand(data_word, 0x00FF);               % LSB
        byte2 = bitshift(bitand(data_word, 0xFF00), -8); % MSB

        % Send the bytes
        write(s, [byte1, byte2], "uint8");

        fprintf('Sent 16-bit word: 0x%04X\n', data_word);
        fprintf('Bytes sent: 0x%02X 0x%02X\n', byte1, byte2);
    catch ME
        fprintf('Error: %s\n', ME.message);
    end

    clear s;
end
