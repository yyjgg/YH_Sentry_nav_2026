// #include <unistd.h>  // UNIX Standard Definitions
// #include <fcntl.h>   // File Control Definitions
// #include <errno.h>   // ERROR Number Definitions
// #include <termios.h> // POSIX Terminal Control Definitions
// #include "go_go_go/SerialPort.h"
// std::mutex ds_mutex;
// Data_Save ds;
// Data_Get dg; // Serial_port
// bool DEBUG_SERIAL =false;
// SerialPort::SerialPort(const char *filename)
// {
//     file_name_ = filename;
//     buadrate_ = B115200;
//     success_ = false;
//     //    serial_mode = NO_INIT;
//     fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port                                           // No terminal will control the process
//     last_fd = fd;
//     if (fd == -1)
//     {
// #ifdef DEBUG_SERIAL
//         printf("open_port wait to open %s .\n", file_name_);
// #endif
//         //  NOTICE("wait serial " << file_name_,1);
//         return;
//     }
//     else if (fd != -1)
//     {
//         fcntl(fd, F_SETFL, 0);
//         // fcntl(fd, F_SETFL, FNDELAY); // no late

// #ifdef DEBUG_SERIAL
//         printf("port is open %s.\n", file_name_);
// #endif
//     }
//     struct termios port_settings;         // structure to store the port settings in
//     cfsetispeed(&port_settings, B115200); // set baud rates

//     cfsetospeed(&port_settings, B115200);
//     port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8; // 8-bit chars
//     // disable IGNBRK for mismatched speed tests; otherwise receive break
//     // as \000 chars
//     port_settings.c_iflag &= ~IGNBRK; // disable break processing
//     port_settings.c_lflag = 0;        // no signaling chars, no echo,
//     // no canonical processing
//     port_settings.c_oflag = 0;                // no remapping, no delays
//     port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
//     port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

//     port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

//     port_settings.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
//     // enable reading
//     port_settings.c_cflag &= ~(PARENB | PARODD); // shut off parity
//     port_settings.c_cflag |= 0;
//     port_settings.c_cflag &= ~CSTOPB;
//     port_settings.c_cflag &= ~CRTSCTS;
//     port_settings.c_iflag = ICANON;

//     port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
//     port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

//     tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

//     //    char write_buffer[]="hello word";
//     //    int bytes_written=0;
//     //    bytes_written=write(fd,write_buffer,sizeof(write_buffer));
// }

// int open_name = 0;

// void SerialPort::restart_serial(void)
// {
//        cout << "test restart !!" << fd << " " << last_fd << endl;
//     close(fd);
//     if (open_name > 10)
//     {
//         open_name = 0;
//     }
//     if (open_name % 2 == 1)
//     {
//         fd = open(SERIAL_PATH, O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port
//     }
//     else
//     {
//         fd = open(SERIAL_PATH, O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port
//     }
//     //    cout << serial_mode << endl;
//     if (fd == -1 && last_fd != -1)
//     {
// #ifdef DEBUG_SERIAL
//         printf("open_port wait to open %s .\n", file_name_);
// #endif
//         //        NOTICE("wait serial",1);
//         last_fd = fd;
//         return;
//     }
//     else if (fd != -1 && last_fd == -1)
//     {
//         fcntl(fd, F_SETFL, 0);
//         // fcntl(fd, F_SETFL, FNDELAY); // no late

//         //        NOTICE("port is open",1);
// #ifdef DEBUG_SERIAL
//         printf("port is open %s.\n", file_name_);
// #endif
//         last_fd = fd;
//     }
//     else
//     {
//         last_fd = fd;
//         return;
//     }
//     struct termios port_settings; // structure to store the port settings in
//         cfsetispeed(&port_settings, B115200); // set baud rates
//         cfsetospeed(&port_settings, B115200);
//     port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8; // 8-bit chars
//     // disable IGNBRK for mismatched speed tests; otherwise receive break
//     // as \000 chars
//     port_settings.c_iflag &= ~IGNBRK; // disable break processing
//     port_settings.c_lflag = 0;        // no signaling chars, no echo,
//     // no canonical processing
//     port_settings.c_oflag = 0;                // no remapping, no delays
//     port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
//     port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

//     port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

//     port_settings.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
//     // enable reading
//     port_settings.c_cflag &= ~(PARENB | PARODD); // shut off parity
//     port_settings.c_cflag |= 0;
//     port_settings.c_cflag &= ~CSTOPB;
//     port_settings.c_cflag &= ~CRTSCTS;
//     port_settings.c_iflag = ICANON;
//     port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
//     port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

//     tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port
// }

// // read the datas,which is enermy color, aim mode and camera vertical angle
// int SerialPort::read_data()
// { 
    
//     tcflush(fd, TCIFLUSH);                /* Discards old data in the rx buffer             */
//     unsigned char read_buffer[READ_SIZE]; /* Buffer to store the data received              */
//     long bytes_read = 0;                  /* Number of bytes read by the read() system call */

//     bytes_read = read(fd, read_buffer, READ_SIZE); /* Read the data   */    

//     if (bytes_read == -1 || bytes_read == 0)
//     {
//         restart_serial();
//     }
//     else
//     {
//         if (read_buffer[0] == 0xAA && read_buffer[1] == 0xBB)
//         {
//             cout << "this is the data I want ! " << endl;
//         }
//         else
//         {
//             if (DEBUG_SERIAL == true)
//             {
//                 cout << "this is not the data I want ! " << endl;
//             }
//         }

//     }
    
// }

// void SerialPort::send_data(const struct Data_Get &data)
// {
//     if (data.size != write(fd, data.raw_data, data.size))
//     {
//         open_name++;

//         if (DEBUG_SERIAL == false)
//         {
//             cout << "!!! send data failure !!!" << fd << "restart fd" << fd << endl;
//         }
//         restart_serial();
//     }
//     else
//     {
//         if (DEBUG_SERIAL == true)
//         {
//             cout << "!!! send data success !!!" << fd << endl;
//         }
//     }
// }

// void Data_Get::Vision2Control(int32_t x, int32_t y)
// {
//     size = 21;

//     raw_data[0] = 0xAA;
//     raw_data[1] = 0xBB;
//     raw_data[2] = (x >> 24) & 0xff;
//     raw_data[3] = (x >> 16) & 0xff;
//     raw_data[4] = (x >> 8) & 0xff;
//     raw_data[5] = (x >> 0) & 0xff;
//     raw_data[6] = (y >> 24) & 0xff;
//     raw_data[7] = (y >> 16) & 0xff;
//     raw_data[8] = (y >> 8) & 0xff;
//     raw_data[9] = (y >> 0) & 0xff;
//     raw_data[20] = 0xBB;
//     cout<<" x_vel "<<x<<" y_vel "<<y<<endl;
    
// }

#include <unistd.h>  // UNIX Standard Definitions
#include <fcntl.h>   // File Control Definitions
#include <errno.h>   // ERROR Number Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include "go_go_go/SerialPort.h"
std::mutex ds_mutex;
Data_Save ds;
Data_Get dg; // Serial_port
bool DEBUG_SERIAL =false;
SerialPort::SerialPort(const char *filename)
{
    file_name_ = filename;
    buadrate_ = B115200;
    success_ = false;
    //    serial_mode = NO_INIT;
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port                                           // No terminal will control the process
    last_fd = fd;
    if (fd == -1)
    {
#ifdef DEBUG_SERIAL
        printf("open_port wait to open %s .\n", file_name_);
#endif
        //  NOTICE("wait serial " << file_name_,1);
        return;
    }
    else if (fd != -1)
    {
        fcntl(fd, F_SETFL, 0);
        // fcntl(fd, F_SETFL, FNDELAY); // no late

#ifdef DEBUG_SERIAL
        printf("port is open %s.\n", file_name_);
#endif
    }
    struct termios port_settings;         // structure to store the port settings in
    cfsetispeed(&port_settings, B115200); // set baud rates

    cfsetospeed(&port_settings, B115200);
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK; // disable break processing
    port_settings.c_lflag = 0;        // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD); // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;

    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

    //    char write_buffer[]="hello word";
    //    int bytes_written=0;
    //    bytes_written=write(fd,write_buffer,sizeof(write_buffer));
}

int open_name = 0;

void SerialPort::restart_serial(void)
{
       cout << "test restart !!" << fd << " " << last_fd << endl;
    close(fd);
    if (open_name > 10)
    {
        open_name = 0;
    }
    if (open_name % 2 == 1)
    {
        fd = open(SERIAL_PATH, O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port
    }
    else
    {
        fd = open(SERIAL_PATH, O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port
    }
    //    cout << serial_mode << endl;
    if (fd == -1 && last_fd != -1)
    {
#ifdef DEBUG_SERIAL
        printf("open_port wait to open %s .\n", file_name_);
#endif
        //        NOTICE("wait serial",1);
        last_fd = fd;
        return;
    }
    else if (fd != -1 && last_fd == -1)
    {
        fcntl(fd, F_SETFL, 0);
        // fcntl(fd, F_SETFL, FNDELAY); // no late

        //        NOTICE("port is open",1);
#ifdef DEBUG_SERIAL
        printf("port is open %s.\n", file_name_);
#endif
        last_fd = fd;
    }
    else
    {
        last_fd = fd;
        return;
    }
    struct termios port_settings; // structure to store the port settings in
        cfsetispeed(&port_settings, B115200); // set baud rates
        cfsetospeed(&port_settings, B115200);
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK; // disable break processing
    port_settings.c_lflag = 0;        // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD); // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port
}

// read the datas,which is enermy color, aim mode and camera vertical angle
int SerialPort::read_data()
{ 
    
    tcflush(fd, TCIFLUSH);                /* Discards old data in the rx buffer             */
    unsigned char read_buffer[READ_SIZE]; /* Buffer to store the data received              */
    long bytes_read = 0;                  /* Number of bytes read by the read() system call */

    bytes_read = read(fd, read_buffer, READ_SIZE); /* Read the data   */    

    if (bytes_read == -1 || bytes_read == 0)
    {
        restart_serial();
    }
    else
    {
        if (read_buffer[0] == 0xAA && read_buffer[1] == 0xBB)
        {
            // Yaw
            int32_t temp_yaw = (static_cast<int32_t>(read_buffer[3]) << 24) | 
                            (static_cast<int32_t>(read_buffer[4]) << 16) | 
                            (static_cast<int32_t>(read_buffer[5]) << 8) | 
                            (static_cast<int32_t>(read_buffer[6]) << 0);
            float yaw = temp_yaw / 10000.0f;
            
            // Pitch
            int32_t temp_pitch = (static_cast<int32_t>(read_buffer[7]) << 24) | 
                                (static_cast<int32_t>(read_buffer[8]) << 16) | 
                                (static_cast<int32_t>(read_buffer[9]) << 8) | 
                                (static_cast<int32_t>(read_buffer[10]) << 0);
            float pitch = temp_pitch / 10000.0f;
            std::cout << "Yaw: " << yaw << "°" << std::endl;
            std::cout << "Pitch: " << pitch << "°" << std::endl;
        }
        else
        {
            if (DEBUG_SERIAL == true)
            {
                cout << "this is not the data I want ! " << endl;
            }
        }

    }
    
}

void SerialPort::send_data(const struct Data_Get &data)
{
    if (data.size != write(fd, data.raw_data, data.size))
    {
        open_name++;

        if (DEBUG_SERIAL == false)
        {
            cout << "!!! send data failure !!!" << fd << "restart fd" << fd << endl;
        }
        restart_serial();
    }
    else
    {
        if (DEBUG_SERIAL == true)
        {
            cout << "!!! send data success !!!" << fd << endl;
        }
    }
}

void Data_Get::Vision2Control(int16_t x, int16_t y)
{
    size = 21;

    raw_data[0] = 0xAA;
    raw_data[1] = 0xBB;
    raw_data[10] = (x >> 8) & 0xff;
    raw_data[11] = (x >> 0) & 0xff;
    raw_data[12] = (y >> 8) & 0xff;
    raw_data[13] = (y >> 0) & 0xff;
    // raw_data[14] = (y >> 24) & 0xff;
    // raw_data[15] = (y >> 16) & 0xff;
    // raw_data[16] = (y >> 8) & 0xff;
    // raw_data[17] = (y >> 0) & 0xff;
    raw_data[20] = 0xBB;
    cout<<" x_vel "<<x<<" y_vel "<<y<<endl;    
}


// int main() 
// {
//     std::cout << "开始测试串口通信..." << std::endl;
//     static auto temp_x = 10000,temp_y = 10000;
    
//     while (1) 
//     { 
//         Data_Get test_data;
//         test_data.Vision2Control(temp_x, temp_y); // 测试数据

//         // 发送数据
//         serial_.send_data(test_data);
        
//         // 读取数据
//         int result = serial_.read_data();
        
        
//         usleep(100000);  
//         temp_x += 2000;
//         temp_y += 2000;

//     }
    
//     std::cout << "测试完成" << std::endl;
//     return 0;
// }