// Definizione dell'enumerazione CommandState
enum CommandType {
    SET, 
    STATE
};

// Struttura Command
struct Command {
    enum CommandType command_type;
    char info[10];
    float value;
};

// Struttura statica per memorizzare l'ultimo comando letto
static struct Command command;
