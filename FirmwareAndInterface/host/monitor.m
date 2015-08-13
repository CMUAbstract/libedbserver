VDD = 3.35; % V
MONITOR_CLK_FREQ = 669 * 32768; % Hz
MONITOR_CLK_PERIOD = 1 / MONITOR_CLK_FREQ;

DELIM_VCAP                  = hex2dec('00');
DELIM_VBOOST                = hex2dec('01');
DELIM_VREG                  = hex2dec('02');
DELIM_VRECT                 = hex2dec('03');
DELIM_VINJ                  = hex2dec('0E');

DELIM_RFRX                  = hex2dec('08');
DELIM_RFTX                  = hex2dec('09');
DELIM_TIME                  = hex2dec('0D');

% RFID command defines
RFID_CMD_QUERYREP           = hex2dec('00');
RFID_CMD_ACK                = hex2dec('40');
RFID_CMD_QUERY              = hex2dec('80');
RFID_CMD_QUERYADJUST        = hex2dec('90');
RFID_CMD_SELECT             = hex2dec('A0');
RFID_CMD_NAK                = hex2dec('C0');
RFID_CMD_REQRN              = hex2dec('C1');
RFID_CMD_READ               = hex2dec('C2');
RFID_CMD_WRITE              = hex2dec('C3');
RFID_CMD_KILL               = hex2dec('C4');
RFID_CMD_LOCK               = hex2dec('C5');
RFID_CMD_ACCESS             = hex2dec('C6');
RFID_CMD_BLOCKWRITE         = hex2dec('C7');
RFID_CMD_BLOCKERASE         = hex2dec('C8');
RFID_CMD_BLOCKPERMALOCK     = hex2dec('C9');
RFID_CMD_READBUFFER         = hex2dec('D2');
RFID_CMD_FILEOPEN           = hex2dec('D3');
RFID_CMD_CHALLENGE          = hex2dec('D4');
RFID_CMD_AUTHENTICATE       = hex2dec('D5');
RFID_CMD_SECURECOMM         = hex2dec('D6');
RFID_CMD_AUTHCOMM           = hex2dec('D7');

RFID_WISP_TX                = hex2dec('09');

rfidPlotTickLabels = {
    RFID_CMD_QUERYREP,          'Query Rep';
    RFID_CMD_ACK,               'ACK';
    RFID_CMD_QUERY,             'Query';
    RFID_CMD_QUERYADJUST,       'Query Adjust';
    RFID_CMD_SELECT,            'Select';
    RFID_CMD_NAK,               'NAK';
    RFID_CMD_REQRN,             'Req RN';
    RFID_CMD_READ,              'Read';
    RFID_CMD_WRITE,             'Write';
    RFID_CMD_KILL,              'Kill';
    RFID_CMD_LOCK,              'Lock';
    RFID_CMD_ACCESS,            'Access';
    RFID_CMD_BLOCKWRITE,        'Block Write';
    RFID_CMD_BLOCKERASE,        'Block Erase';
    RFID_CMD_BLOCKPERMALOCK,    'Block Permalock';
    RFID_CMD_READBUFFER,        'Read Buffer';
    RFID_CMD_FILEOPEN,          'File Open';
    RFID_CMD_CHALLENGE,         'Challenge';
    RFID_CMD_AUTHENTICATE,      'Authenticate';
    RFID_CMD_SECURECOMM,        'Secure Comm';
    RFID_CMD_AUTHCOMM,          'Auth Comm';
    RFID_WISP_TX,               'WISP Tx';
};