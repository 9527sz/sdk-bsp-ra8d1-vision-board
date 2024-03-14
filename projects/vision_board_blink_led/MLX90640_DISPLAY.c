#include <rtthread.h>
#include <rtdevice.h>
#include "MLX90640_DISPLAY.h"

#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06

#define  MLX90640_ADDR 0x33
#define	 RefreshRate FPS8HZ
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air

#define SRC_HEIGHT 24
#define SRC_WIDTH 32
#define DEST_HEIGHT 240
#define DEST_WIDTH 320

static uint16_t eeMLX90640[832];
static float mlx90640To[768];
static int frame[834];
float src_matrix[24][32];
float dest_matrix[240][320];
float emissivity = 0.95;
int status;
paramsMLX90640 mlx90640;

int MLX90640_INIT(void)
{
    MLX90640_SetRefreshRate(MLX90640_ADDR, RefreshRate);
    MLX90640_SetChessMode(MLX90640_ADDR);
    status = MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640); //读取像素矫正参数
    if(status != 0) rt_kprintf("\r\nload system parameters error with code:%d\r\n", status);
    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640); //解析矫正参数
    if(status != 0) rt_kprintf("\r\nParameter extraction failed with error code:%d\r\n", status);
}
INIT_DEVICE_EXPORT(MLX90640_INIT);

void bilinear_interpolation(float src_matrix[SRC_HEIGHT][SRC_WIDTH], float dest_matrix[DEST_HEIGHT][DEST_WIDTH])
{
    float x_ratio = (float)(SRC_WIDTH - 1) / (float)(DEST_WIDTH - 1);
    float y_ratio = (float)(SRC_HEIGHT - 1) / (float)(DEST_HEIGHT - 1);

    for(int y = 0; y < DEST_HEIGHT; y++)
    {
        for(int x = 0; x < DEST_WIDTH; x++)
        {
            float x_src = x * x_ratio;
            float y_src = y * y_ratio;

            int x1 = (int)x_src;
            int y1 = (int)y_src;
            int x2 = x1 + 1;
            int y2 = y1 + 1;

            float x_diff = x_src - x1;
            float y_diff = y_src - y1;

            // Bilinear interpolation
            float top_left = src_matrix[y1][x1] * (1 - x_diff) * (1 - y_diff);
            float top_right = src_matrix[y1][x2] * x_diff * (1 - y_diff);
            float bottom_left = src_matrix[y2][x1] * (1 - x_diff) * y_diff;
            float bottom_right = src_matrix[y2][x2] * x_diff * y_diff;

            dest_matrix[y][x] = top_left + top_right + bottom_left + bottom_right;
        }
    }
}
float max_temperature;
void find_max_temperature(float src_matrix[SRC_HEIGHT][SRC_WIDTH], float dest_matrix[DEST_HEIGHT][DEST_WIDTH], int* max_dest_row, int* max_dest_col)
{
    // 找到最高温度及其位置
    max_temperature = src_matrix[0][0];
    int max_row = 0;
    int max_col = 0;
    for(int row = 0; row < SRC_HEIGHT; row++)
    {
        for(int col = 0; col < SRC_WIDTH; col++)
        {
            if(src_matrix[row][col] > max_temperature)
            {
                max_temperature = src_matrix[row][col];
                max_row = row;
                max_col = col;
            }
        }
    }
    // 将最高温度值复制到目标矩阵的相应位置
    *max_dest_row = max_row * (DEST_HEIGHT / SRC_HEIGHT);  // 目标矩阵中的行索引
    *max_dest_col = max_col * (DEST_WIDTH / SRC_WIDTH);    // 目标矩阵中的列索引
    dest_matrix[*max_dest_row][*max_dest_col] = max_temperature;
}
float min_temperature;
void find_min_temperature(float src_matrix[SRC_HEIGHT][SRC_WIDTH], float dest_matrix[DEST_HEIGHT][DEST_WIDTH], int* min_dest_row, int* min_dest_col)
{
    // 找到最高温度及其位置
    min_temperature = src_matrix[0][0];
    int min_row = 0;
    int min_col = 0;
    for(int row = 0; row < SRC_HEIGHT; row++)
    {
        for(int col = 0; col < SRC_WIDTH; col++)
        {
            if(src_matrix[row][col] < min_temperature)
            {
				min_temperature = src_matrix[row][col];
                min_row = row;
                min_col = col;
            }
        }
    }
    // 将最高温度值复制到目标矩阵的相应位置
    *min_dest_row = min_row * (DEST_HEIGHT / SRC_HEIGHT);  // 目标矩阵中的行索引
    *min_dest_col = min_col * (DEST_WIDTH / SRC_WIDTH);    // 目标矩阵中的列索引
    dest_matrix[*min_dest_row][*min_dest_col] = min_temperature;
}

struct drv_lcd_device
{
    struct rt_device parent;
    struct rt_device_graphic_info lcd_info;
};

uint16_t calculate_color(float temperature)
{
    // 将温度线性映射到 RGB565 颜色范围
    int red = (int)((temperature / 7.0) * 31) << 11; // 红色占用高 5 位
    int green = 5; // 可根据需要调整
    int blue = 31 - (int)((temperature / 90.0) * 30); // 蓝色占用低 5 位

    // 合并颜色分量并返回 RGB565 值
    return (red & 0xF800) | ((green & 0x07E0) << 5) | (blue & 0x001F);
}



#define THREAD_PRIORITY         25
#define THREAD_STACK_SIZE       16384
#define THREAD_TIMESLICE        5
static rt_thread_t tid1 = RT_NULL;
static void thread1_entry(void* parameter)
{
    int max_dest_row, max_dest_col;
	int min_dest_row, min_dest_col;
    struct drv_lcd_device* lcd;
    struct rt_device_rect_info rect_info;
    rect_info.x = 0;
    rect_info.y = 0;
    rect_info.width = LCD_WIDTH;
    rect_info.height = LCD_HEIGHT;

    lcd = (struct drv_lcd_device*)rt_device_find("lcd");

    /* red */
    for(int i = 0; i < LCD_BUF_SIZE / 2; i++)
    {
        lcd->lcd_info.framebuffer[2 * i] = 0xFF;
        lcd->lcd_info.framebuffer[2 * i + 1] = 0xFF;
    }

    lcd->parent.control(&lcd->parent, RTGRAPHIC_CTRL_RECT_UPDATE, &rect_info);
    rt_thread_mdelay(1000);




    uint16_t color = 0xF800; // 16-bit RGB565 color (e.g., red)

    lcd_gpu_fill_array(0, 0, 240, 180, &color);

    while(1)
    {
        int status = MLX90640_GetFrameData(MLX90640_ADDR, frame);  //读取一帧原始数据
        if(status < 0)
        {
            rt_kprintf("GetFrame Error: %d\r\n", status);
        }
        float vdd = MLX90640_GetVdd(frame, &mlx90640);  //计算 Vdd（这句可有可无）
        float Ta = MLX90640_GetTa(frame, &mlx90640);    //计算实时外壳温度
        float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature



        MLX90640_CalculateTo(frame, &mlx90640, emissivity , tr, mlx90640To);            //计算像素点温度
        MLX90640_BadPixelsCorrection(mlx90640.brokenPixels, mlx90640To, 1, &mlx90640);  //坏点处理
        MLX90640_BadPixelsCorrection(mlx90640.outlierPixels, mlx90640To, 1, &mlx90640); //坏点处理


//    rt_kprintf("\r\n==========================I==========================\r\n");
//    for(int i = 0; i < 768; i++)
//    {
//        if(i % 32 == 0 && i != 0)
//        {
//            rt_kprintf("\r\n");
//        }
//        printf("%2.0f   ", mlx90640To[i]);
//    }
//    rt_kprintf("\r\n==========================I==========================\r\n");


//一维数组转换为二维数组
        int index = 0;
        for(int i = 0; i < 24; i++)
        {
            for(int j = 0; j < 32; j++)
            {
                src_matrix[i][j] = mlx90640To[index++];
            }
        }
        bilinear_interpolation(src_matrix, dest_matrix);

        find_max_temperature(src_matrix, dest_matrix, &max_dest_row, &max_dest_col);
		find_min_temperature(src_matrix, dest_matrix, &min_dest_row, &min_dest_col);

        rt_thread_mdelay(5);

        for(int row = 0; row < 240; row++)
        {
            for(int col = 0; col < 320; col++)
            {
                // 计算温度值在 RGB565 格式的颜色
                float temperature = dest_matrix[row][col];
                uint16_t color = calculate_color(temperature);
                // 在屏幕上绘制像素
                lcd_draw_pixel(col + 80,  row + 60, color);
            }
        }


        if(max_dest_row > 5 && max_dest_row < 235 && max_dest_col > 5 && max_dest_col < 315)
        {
            for(int cross_row = max_dest_row - 5; cross_row < max_dest_row + 5; cross_row++)
            {
                lcd_draw_pixel(max_dest_col + 80, cross_row + 60, 0xFFFF);
            }
            for(int cross_col = max_dest_col - 5; cross_col < max_dest_col + 5; cross_col++)
            {
                lcd_draw_pixel(cross_col + 80, max_dest_row + 60, 0xFFFF);
            }
        }
        if(min_dest_row > 5 && min_dest_row < 235 && min_dest_col > 5 && min_dest_col < 315)
        {
            for(int cross_row = min_dest_row - 5; cross_row < min_dest_row + 5; cross_row++)
            {
                lcd_draw_pixel(min_dest_col + 80, cross_row + 60, 0xFFFF);
            }
            for(int cross_col = min_dest_col - 5; cross_col < min_dest_col + 5; cross_col++)
            {
                lcd_draw_pixel(cross_col + 80, min_dest_row + 60, 0xFFFF);
            }
        }
        LCD_ShowString(30, 315, "MAX_temperature:", BLACK, BLACK, 32, 1);
        LCD_ShowFloatNum1(325, 320, max_temperature, 2, BLACK, BLACK, 24);
        LCD_ShowIntNum(300, 320, max_temperature, 2, BLACK, BLACK, 24);
		
		LCD_ShowString(30, 0, "MIN_temperature:", BLACK, BLACK, 32, 1);
        LCD_ShowFloatNum1(325, 5, min_temperature, 2, BLACK, BLACK, 24);
        LCD_ShowIntNum(300, 5, min_temperature, 2, BLACK, BLACK, 24);
    }
}

int thread_sample(void)
{
    /* 创建线程 1，名称是 thread1，入口是 thread1_entry*/
    tid1 = rt_thread_create("thread1",
                            thread1_entry, RT_NULL,
                            THREAD_STACK_SIZE,
                            THREAD_PRIORITY, THREAD_TIMESLICE);

    /* 如果获得线程控制块，启动这个线程 */
    if(tid1 != RT_NULL)
        rt_thread_startup(tid1);
}

MSH_CMD_EXPORT(thread_sample, thread sample);


/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t temp, sizex, t, m = 0;
    uint16_t i, TypefaceNum; //一个字符所占字节大小
    uint16_t x0 = x;
    sizex = sizey / 2;
    TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
    num = num - ' '; //得到偏移后的值
    for(i = 0; i < TypefaceNum; i++)
    {
        if(sizey == 12)temp = ascii_1206[num][i];		   //调用6x12字体
        else if(sizey == 16)temp = ascii_1608[num][i];		 //调用8x16字体
        else if(sizey == 24)temp = ascii_2412[num][i];		 //调用12x24字体
        else if(sizey == 32)temp = ascii_3216[num][i];		 //调用16x32字体
        else return;
        for(t = 0; t < 8; t++)
        {
            if(!mode)//非叠加模式
            {

                m++;
                if(m % sizex == 0)
                {
                    m = 0;
                    break;
                }
            }
            else//叠加模式
            {
                if(temp & (0x01 << t))lcd_draw_pixel(x, y, fc); //画一个点
                x++;
                if((x - x0) == sizex)
                {
                    x = x0;
                    y++;
                    break;
                }
            }
        }
    }
}
/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t* p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    while(*p != '\0')
    {
        LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
        x += sizey / 2;
        p++;
    }
}
/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
uint32_t mypow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while(n--)result *= m;
    return result;
}

/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t t, temp, sizex;
    uint16_t num1;
    sizex = sizey / 2;
    num1 = num * 100;
    for(t = 0; t < len; t++)
    {
        temp = (num1 / mypow(10, len - t - 1)) % 10;
        if(t == (len - 2))
        {
            for(int col = x-40; col < x + 45; col++)
            {
                for(int row = y + 25; row > y; row--)
                {
                    lcd_draw_pixel(col, row, WHITE);
                }
            }
            LCD_ShowChar(x + (len - 2)*sizex, y, '.', fc, bc, sizey, 1);
            t++;
            len += 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 1);
    }
}

/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowIntNum(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    uint8_t sizex = sizey / 2;
    uint16_t num1;
    num1 = num * 100 / 100;
    for(t = 0; t < len; t++)
    {
        temp = (num1 / mypow(10, len - t - 1)) % 10;
        if(enshow == 0 && t < (len - 1))
        {
            if(temp == 0)
            {
                LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 1);
                continue;
            }
            else enshow = 1;

        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 1);
    }
}

