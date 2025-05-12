#include "bot_ultrasonic.hpp"

bool BotUltrasonic::init()
{
    ScheduleOnInterval(2000_us);
    return true;
}

void BotUltrasonic::Run()
{
    perf_begin(_loop_perf);
    perf_count(_loop_interval_perf);

    UpdateData();

    //无论上位机设置超声波关闭还是开启，都上报超声波数据，但是如果是关闭状态，在触发阈值时底盘不停车
    read_data();

    _bot_distance_sensor_pub.publish(bot_distance_sensor_pub);

    perf_end(_loop_perf);
}

void BotUltrasonic::UpdateData(){
  if (_bot_sensor_event_sub.updated()) {
    _bot_sensor_event_sub.copy(&bot_sensor_event_sub);
  }
}

BotUltrasonic::BotUltrasonic() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test2),
    uart_fd(-1)
{
}

BotUltrasonic::~BotUltrasonic()
{
    close_uart();
    perf_free(_loop_perf);
    perf_free(_loop_interval_perf);
}


bool BotUltrasonic::init_uart(const char *device, int baud_rate)
{
    uart_fd = ::open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        PX4_ERR("Failed to open UART device: %s", device);
        return false;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 添加非规范模式设置
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控
    options.c_oflag &= ~OPOST; // 原始输出

    // 应用更改
    if (tcsetattr(uart_fd, TCSANOW, &options) != 0) {
        PX4_ERR("Failed to set UART attributes");
        return false;
    }
    return true;
}

void BotUltrasonic::set_work_mode()
{
    // 清空UART缓冲区
    tcflush(uart_fd, TCIOFLUSH);

    uint8_t set_mode_cmd[8] = {0x01, 0x06, 0x02, 0x02, 0x00, 0x00}; // 设置为UART受控输出模式
    uint16_t crc = calculate_crc(set_mode_cmd, 6);
    set_mode_cmd[6] = crc & 0xFF;
    set_mode_cmd[7] = (crc >> 8) & 0xFF;

    write(uart_fd, set_mode_cmd, sizeof(set_mode_cmd));

    uint8_t buffer[8];
    read(uart_fd, buffer, sizeof(buffer));

    tcflush(uart_fd, TCIOFLUSH);

    uint8_t set_mode[8] = {0x01, 0x06, 0x02, 0x16, 0x00, 0x01}; // 设置为同时工作模式
    uint16_t crc01 = calculate_crc(set_mode, 6);
    set_mode[6] = crc01 & 0xFF;
    set_mode[7] = (crc01 >> 8) & 0xFF;

    write(uart_fd, set_mode, sizeof(set_mode));

    uint8_t buffer01[8];
    read(uart_fd, buffer01, sizeof(buffer01));
}

void BotUltrasonic::read_data()
{
    if (uart_fd == -1 && !init_uart("/dev/ttyS3", B9600)) {
        PX4_ERR("UART initialization failed");
        return;
    }

    uint8_t trigger_cmd[1] = {0xff}; // 触发传感器
    int write_ret = write(uart_fd, trigger_cmd, sizeof(trigger_cmd));
    if (write_ret != sizeof(trigger_cmd)) {
        PX4_ERR("Failed to write trigger command to UART");
        return;
    }

    fd_set read_fds;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100毫秒

    uint8_t buffer[10];
    int total_read_bytes = 0;
    int retries = 0;
    const int max_retries = 5;
    enum ParseState { WAIT_FOR_HEADER, READ_DATA } state = WAIT_FOR_HEADER;

    while (retries < max_retries) {
        FD_ZERO(&read_fds);
        FD_SET(uart_fd, &read_fds);

        int select_ret = select(uart_fd + 1, &read_fds, NULL, NULL, &timeout);
        if (select_ret > 0 && FD_ISSET(uart_fd, &read_fds)) {
            uint8_t byte;
            int read_ret = read(uart_fd, &byte, 1);
            if (read_ret > 0) {
                switch (state) {
                    case WAIT_FOR_HEADER:
                        if (byte == 0xFF) {
                            buffer[0] = byte;
                            total_read_bytes = 1;
                            state = READ_DATA;
                        }
                        break;

                    case READ_DATA:
                        buffer[total_read_bytes++] = byte;
                        if (total_read_bytes == sizeof(buffer)) {
                            state = WAIT_FOR_HEADER;
                            uint16_t distance1 = (buffer[1] << 8) | buffer[2];
                            uint16_t distance2 = (buffer[3] << 8) | buffer[4];
                            uint16_t distance3 = (buffer[5] << 8) | buffer[6];
                            uint16_t distance4 = (buffer[7] << 8) | buffer[8];
                            uint8_t checksum = buffer[9];
                            uint8_t calculated_checksum = 0;
                            for (int i = 0; i < 9; i++) {
                                calculated_checksum += buffer[i];
                            }
                            calculated_checksum &= 0xFF;

                            if (checksum == calculated_checksum) {
                                bot_distance_sensor_pub.distance1 = distance1;
                                bot_distance_sensor_pub.distance2 = distance2;
                                bot_distance_sensor_pub.distance3 = distance3;
                                bot_distance_sensor_pub.distance4 = distance4;
                                return;
                            }

                        }
                        break;
                }
            } else {
                retries++;
            }
        } else {
            retries++;
        }
    }
}



int BotUltrasonic::task_spawn(int argc, char *argv[])
{
    BotUltrasonic *instance = new BotUltrasonic();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }
    }

    delete instance;
    _object.store(nullptr);
    return PX4_ERROR;
}

int BotUltrasonic::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int BotUltrasonic::print_status()
{
    PX4_INFO("Ultrasonic Module running");
    return 0;
}

int BotUltrasonic::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Module to read data from ultrasonic sensors via UART and publish distance_sensor messages for obstacle avoidance.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("bot_ultrasonic", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

void BotUltrasonic::close_uart()
{
    if (uart_fd != -1) {
        ::close(uart_fd);
        uart_fd = -1;
    }
}

uint16_t BotUltrasonic::calculate_crc(uint8_t *buffer, int length)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

extern "C" __EXPORT int bot_ultrasonic_main(int argc, char *argv[])
{
    return BotUltrasonic::main(argc, argv);
}
