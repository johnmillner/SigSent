#include <SPI.h>

// This flag is true if we are executing a command
byte doing_command = 0;
byte message_data = -1;

// These definitions came from the pi_spi_modules.py file that define the message
// class and its generation functions :)
byte mode_change_header = 0b00110000;
byte walking_move_header = 0b00001100;
byte esc_header = 0b00000011;
byte fwd = 0b00000000;
byte left = 0b00001111;
byte right = 0b11110000;
byte back = 0b11111111;
byte driving_mode = 0b00001111;
byte walking_mode = 0b11110000;

// Lets the MCU to know what to expect for the next message to parse
// it correctly
byte receiving_header = 1;
byte receiving_mode = 0;
byte receiving_walking_move = 0;
byte receiving_esc_direction = 0;
byte receiving_esc_speed = 0;

enum ModeValue
{
    WALKING_MODE,
    DRIVING_MODE,
    MODE_ERROR
};

enum Direction
{
    FWD,
    LEFT,
    RIGHT,
    BACK,
    DIR_ERROR
};

enum MessageType
{
    MODE_TYPE,
    WALKING_TYPE,
    ESC_TYPE,
    MESSAGE_ERROR
};

Direction esc_dir;

ModeValue get_mode(byte mode)
{
    if (mode == walking_mode)
        return WALKING_MODE;
    if (mode == driving_mode)
        return DRIVING_MODE;
    
    return MODE_ERROR;
}

MessageType get_message_type(byte header)
{
    if (header == mode_change_header)
        return MODE_TYPE;
    if (header == walking_move_header)
        return WALKING_TYPE;
    if (header == esc_header)
        return ESC_TYPE;

    return MESSAGE_ERROR;
}

Direction get_direction(byte dir)
{
    if (dir == fwd)
        return FWD;
    if (dir == left)
        return LEFT;
    if (dir == right)
        return RIGHT;
    if (dir == back)
        return BACK;
    
    return DIR_ERROR;
}

//#####################################################
//# Josh, the three functions below are ALL YOU BABY  #
//# switch_mode, walk_move, drive_escs                #
//#####################################################
int switch_mode(ModeValue mode)
{
    // Depending on the mode, do some stuff to the servos

    // Return 1 for success, 0 for failure
    return 1;
}

int walk_move(Direction dir)
{
    return 1;
}

// We don't have to have a parameter since the direction value is
// a global value that was set in the previous SPI interrupt, but I
// think its more readable to know exactly is being passed to it when tracing the caller
int drive_escs(Direction dir, int speed)
{
    return 1;
}

ISR (SPI_STC_vect)
{
    // grab byte from SPI Data Register
    byte data = SPDR;
    
    //Serial.print("Received: ");
    Serial.println(data);

    if (receiving_header)
    {
        //Serial.println("Receiving header");
        MessageType message_type = get_message_type(data);
        
        if (message_type == MESSAGE_ERROR)
            return;
        
        receiving_header = 0;

        if (message_type == MODE_TYPE)
            receiving_mode = 1;
        else if (message_type == WALKING_TYPE)
            receiving_walking_move = 1;
        else if (message_type == ESC_TYPE)
            receiving_esc_direction = 1;
    }

    else if (receiving_mode)
    {
        ModeValue mode = get_mode(data);

        if (mode == MODE_ERROR)
        {
            // Failure, start over
            receiving_header = 1;
            receiving_mode = 0;

            return;
        }

        // Got the mode, now do the crazy servo code that switches mode
        byte ret = switch_mode(mode);

        if (ret == 0)
        {
            ;// Failed to switch mode, how do we handle this?
        }

        receiving_mode = 0;
        receiving_header = 1;
    }   

    else if (receiving_walking_move || receiving_esc_direction)
    {
        //Serial.println("Receiving direction");
        Direction dir = get_direction(data);

        if (dir == DIR_ERROR)
        {
            if (receiving_walking_move)
                receiving_walking_move = 0;
            else
                receiving_esc_direction = 0;

            receiving_header = 1;
        }

        if (receiving_walking_move)
        {
            byte ret = walk_move(dir);

            if (ret == 0)
            {
                ;
            }

            receiving_header = 1;
            receiving_walking_move = 0;
        }
        
        else if (receiving_esc_direction)
        {
            esc_dir = dir;
            receiving_esc_direction = 0;
            receiving_esc_speed = 1;
        }
    }

    else if (receiving_esc_speed)
    {
        doing_command = 1;
        receiving_header = 1;
    }
}

void setup() 
{
    // have to send on master in, *slave out*
    pinMode(MISO, OUTPUT);

    // turn on SPI in slave mode and attach the interrupt
    SPCR |= _BV(SPE);
    SPI.attachInterrupt();

    Serial.begin(9600);      // open the serial port at 9600 bps:    

}

void loop() 
{

    while(!doing_command)
    {
        // Do nothing
        ready_flag = 0xFF;
    }

    while (message_data == -1)
    {
        // Wait for message from PI
    }

    if (doing_command && receiving_header)
    {
        Serial.println("Receiving header");
        MessageType message_type = get_message_type(message_data);

        if (message_type == MESSAGE_ERROR)
        {
            // ABORT
            receiving_header = 0;
            doing_command = 0;
            message_data = -1;
        }
        else
        {
            receiving_header = 0;

            if (message_type == MODE_TYPE)
                receiving_mode = 1;
            else if (message_type == WALKING_TYPE)
                receiving_walking_move = 1;
            else if (message_type == ESC_TYPE)
                receiving_esc_direction = 1;
            
            message_data = -1;
        }
    }

    else if (doing_command && receiving_mode)
    {
        // This shouldnt be necessary, but just in case...
        while (message_data == -1)
        {
            // Wait for message from PI
        }

        ModeValue mode = get_mode(data);

        if (mode == MODE_ERROR)
        {
            // Anything special here?
        }
        else
        {
            // Got the mode, now do the crazy servo code that switches mode
            byte ret = switch_mode(mode);

            if (ret == 0)
            {
                ;// Failed to switch mode, how do we handle this?
            }
        }

        // Whether we fail or succeed, no more to be done, go back into idle mode
        receiving_mode = 0;
        doing_command = 0;
        message_data = -1;
    }

    else if (doing_command && (receiving_walking_move || receiving_esc_direction))
    {
        // This shouldnt be necessary, but just in case...
        while (message_data == -1)
        {
            // Wait for message from PI
        }

        //Serial.println("Receiving direction");
        Direction dir = get_direction(data);

        if (dir == DIR_ERROR)
        {
            if (receiving_walking_move)
                receiving_walking_move = 0;
            else
                receiving_esc_direction = 0;

            doing_command = 0;
            message_data = -1;
        }

        if (receiving_walking_move)
        {
            byte ret = walk_move(dir);

            if (ret == 0)
            {
                ;
            }

            receiving_header = 1;
            receiving_walking_move = 0;
        }

        else if (receiving_esc_direction)
        {
            esc_dir = dir;
            receiving_esc_direction = 0;
            receiving_esc_speed = 1;
        }
    }

    else if (doing_command && receiving_esc_speed)
    {
        // This shouldnt be necessary, but just in case...
        while (message_data == -1)
        {
            // Wait for message from PI
        }

        //Serial.println("Receiving speed");
        // speed is just a plain old 8 bit number, no processing needed, just do it
        byte ret = drive_escs(esc_dir, data);

        if (ret == 0)
        {
            ;
        }

        receiving_header = 1;
        receiving_esc_speed = 0;
        esc_dir = DIR_ERROR;
    }
}
