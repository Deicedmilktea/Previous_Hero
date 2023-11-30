#include "UI_task.h"
#include "CRC.h"
#include "judge.h"
uint8_t UI_Seq;                         //包序号

uint8_t referee_uart_tx_buf[2][150];    //裁判系统学生串口发送DMA缓冲池
uint8_t referee_tx_fifo = 0;            //正在使用的缓存池


extern JUDGE_MODULE_DATA Judge_Hero;
extern UART_HandleTypeDef huart6;
static uint8_t usart6_tx_dma_is_busy = 0;


#define UI_NEED_REFRESH (userUI_control.module_refresh_timer > USERUI_MODULE_REFRESH_TIME)
#define UI_IS_EXTERN    (userUI_control.module_extern_flag)


/**
  * @brief          自定义UI初始化，数据初始化，绘制背景元素
  * @param[in]      none
  * @retval         none
  */
void userUI_init(void);

/**
  * @brief          自定义UI绘制背景元素
  * @param[in]      none
  * @retval         none
  */
void userUI_draw_background(void);

/**
  * @brief          自定义UI绘制密位瞄准靶
  * @param[in]      水平补偿（左负右正）
  * @param[in]      垂直补偿（上正下负）
  * @param[in]      缩放比例
  * @retval         none
  */
void userUI_draw_foresight(int16_t delta_x, int16_t delta_y, fp32 scale);


//将要显示的图形
Graph_Data graph1, graph2, graph3, graph4, graph5, graph6, graph7;
//将要显示的字符串
String_Data ui_str;

userUI_control_t userUI_control;

/**
  * @brief          自定义UI任务，在client上绘制自定义UI
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void UI_Task(void const* argument)
{

     osDelay(USERUI_TASK_CONTROL_TIME_MS);
  
    //初始化
    userUI_init();
    
    while (1)
    {
          userUI_init();
        osDelay(USERUI_TASK_CONTROL_TIME_MS);
    }
}


/**
  * @brief          自定义UI初始化，数据初始化，绘制背景元素
  * @param[in]      none
  * @retval         none
  */
void userUI_init(void)
{
    //userUI_control.rc = get_remote_control_point();
    userUI_control.module_refresh_timer = 0;
    userUI_control.module_extern_flag = 0;

    //绘制密位靶（图层1）
    userUI_draw_foresight(0, 0, 1.0f);
}




/**
  * @brief          自定义UI绘制密位瞄准靶（画在图层1上）
  * @param[in]      水平补偿（右正左负）
  * @param[in]      垂直补偿（上正下负）
  * @param[in]      缩放比例（保留）
  * @retval         none
  */
void userUI_draw_foresight(int16_t delta_x, int16_t delta_y, fp32 scale)
{
    Line_Draw(&graph1, "FS1", UI_Graph_ADD, 1, UI_Color_Cyan, 1, 960 + delta_x, 370 + delta_y, 960 + delta_x, 580 + delta_y);
    Line_Draw(&graph2, "FS2", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 900 + delta_x, 540 + delta_y, 1020 + delta_x, 540 + delta_y);
    Line_Draw(&graph3, "FS3", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 920 + delta_x, 510 + delta_y, 1000 + delta_x, 510 + delta_y);
    Line_Draw(&graph4, "FS4", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 930 + delta_x, 470 + delta_y, 990 + delta_x, 470 + delta_y);
    Line_Draw(&graph5, "FS5", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 940 + delta_x, 420 + delta_y, 980 + delta_x, 420 + delta_y);
    Line_Draw(&graph6, "FS6", UI_Graph_ADD, 1, UI_Color_Cyan, 2, 950 + delta_x, 380 + delta_y, 970 + delta_x, 380 + delta_y);
    Circle_Draw(&graph7, "FS7", UI_Graph_ADD, 1, UI_Color_Cyan, 1, 960 + delta_x, 540 + delta_y, 20);
    ui_display_7_graph(&graph1, &graph2, &graph3, &graph4, &graph5, &graph6, &graph7);
}










/**
 *  @brief          使用自定义UI绘制直线
 *  @param[out]     Graph_Data*
 *  @param[in]      图形名称 三个字符
 *  @param[in]      图形操作
 *  @param[in]      图层 0-9
 *  @param[in]      图形颜色
 *  @param[in]      图形线宽
 *  @param[in]      起始X坐标
 *  @param[in]      起始Y坐标
 *  @param[in]      结束X坐标
 *  @param[in]      结束Y坐标
 *  @retval         none
 */
void Line_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->operate_tpye = Graph_Operate;
    image->graphic_tpye = UI_Graph_Line;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}


/**
 *  @brief          使用自定义UI绘制矩形
 *  @author         山东理工大学
 *  @param[out]     Graph_Data*
 *  @param[in]      图形名称 三个字符
 *  @param[in]      图形操作
 *  @param[in]      图层 0-9
 *  @param[in]      图形颜色
 *  @param[in]      图形线宽
 *  @param[in]      起始X坐标
 *  @param[in]      起始Y坐标
 *  @param[in]      结束X坐标
 *  @param[in]      结束Y坐标
 */
void Rectangle_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Rectangle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}


/**
 *  @brief          使用自定义UI绘制正圆
 *  @author         山东理工大学
 *  @param[out]     Graph_Data*
 *  @param[in]      图形名称 三个字符
 *  @param[in]      图形操作
 *  @param[in]      图层 0-9
 *  @param[in]      图形颜色
 *  @param[in]      图形线宽
 *  @param[in]      圆心X坐标
 *  @param[in]      圆心Y坐标
 *  @param[in]      半径
 *  @retval         none
 */
void Circle_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Circle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->radius = Graph_Radius;
}


/**
 *  @brief          使用自定义UI绘制圆弧
 *  @author         山东理工大学
 *  @param[out]     Graph_Data*
 *  @param[in]      图形名称 三个字符
 *  @param[in]      图形操作
 *  @param[in]      图层 0-9
 *  @param[in]      图形颜色
 *  @param[in]      图形线宽
 *  @param[in]      起始角度
 *  @param[in]      中止角度
 *  @param[in]      圆心X坐标
 *  @param[in]      圆心Y坐标
 *  @param[in]      X方向轴长
 *  @param[in]      Y方向轴长，参考椭圆的使用方法
 *  @retval         none
 */
void Arc_Draw(Graph_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t x_Length, uint32_t y_Length)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Arc;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_StartAngle;
    image->end_angle = Graph_EndAngle;
    image->end_x = x_Length;
    image->end_y = y_Length;
}



/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    小数位数
        Start_x、Start_x    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/

void Float_Draw(Float_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, float Graph_Float)
{
    int i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->graphic_name[2 - i] = imagename[i];
    image->graphic_tpye = UI_Graph_Float;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_Size;
    image->end_angle = Graph_Digit;
    image->graph_Float = Graph_Float;
}


/**
 *  @brief          绘制字符型数据
 *  @param[in]      String_Data* 字符串结构体的指针
 *  @param[in]      图片名称
 *  @param[in]      图形操作
 *  @param[in]      图层 [0,9]
 *  @param[in]      图形颜色
 *  @param[in]      图形线宽
 *  @param[in]      字号，建议与线宽的比为 10:1
 *  @param[in]      开始坐标X
 *  @param[in]      开始坐标Y
 *  @param[in]      待发送的字符串，一次最多发送30个字符
 *  @retval         none
 */
void string_Draw(String_Data* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Start_x, uint32_t Start_y, char* Char_Data)
{
    uint8_t i;

    for (i = 0; i < 3 && imagename[i] != '\0'; i++)
        image->Graph_Control.graphic_name[2 - i] = imagename[i];
    image->Graph_Control.graphic_tpye = UI_Graph_Char;
    image->Graph_Control.operate_tpye = Graph_Operate;
    image->Graph_Control.layer = Graph_Layer;
    image->Graph_Control.color = Graph_Color;
    image->Graph_Control.start_x = Start_x;
    image->Graph_Control.start_y = Start_y;
    image->Graph_Control.width = Graph_Size;                //线宽
    image->Graph_Control.start_angle = Graph_Digit;         //字号

    i = 0;
    while (Char_Data[i] != '\0' && i < 30)
    {
        image->show_Data[i] = Char_Data[i];
        i++;
    }
    image->Graph_Control.end_angle = i;                     //记录字符长度
    while (i < 30)
    {
        image->show_Data[i++] = 0;
    }
    
}


/**
 *  @brief          删除图层
 *  @param[in]      操作类型  0：空操作   1：删除指定图层   2：删除所有图层
 *  @param[in]      图层数 [0, 9]
 */
void UI_delete(uint8_t operate_type, uint8_t layer)
{
    UI_Packhead head;
    UI_Data_Operate operate;
    UI_Data_Delete delete_layer;

    //1. 帧头内容初始化
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(UI_Data_Delete);
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //2. 机器人间通讯包头初始化
    operate.Data_ID = 0x100;                //自定义ID，删除图层
    operate.Sender_ID = get_robot_id();     //发送端ID
    operate.Receiver_ID = get_client_id();  //接收端ID

    //3. 删除操作转录
    delete_layer.Delete_Operate = operate_type;
    delete_layer.Layer = layer;

    //4. 数据转存
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), &delete_layer, sizeof(UI_Data_Delete));

    //5. CRC校验
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(UI_Data_Delete) + 2);

    //6. DMA发送
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(UI_Data_Delete) + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}


/**
 *  @brief          使用自定义UI绘制一个图形，刷新出来在屏幕上显示
 *  @param[in]      图形
 *  @return         none
 */
void ui_display_1_graph(Graph_Data* graph1)
{
    UI_Packhead head;
    UI_Data_Operate operate;

    //帧头内容初始化
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(Graph_Data);  //应该为6+15
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //机器人间通讯包头初始化
    operate.Data_ID = 0x101;                //自定义ID
    operate.Sender_ID = get_robot_id();     //发送端ID
    operate.Receiver_ID = get_client_id();  //接收端ID

    //数据转存
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), graph1, sizeof(Graph_Data));

    //CRC校验
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) + 2);

    //DMA发送
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}


/**
 *  @brief          使用自定义UI绘制两个图形，刷新出来在屏幕上显示
 *  @param[in]      图形1
 *  @param[in]      图形2
 *  @return         none
 */
void ui_display_2_graph(Graph_Data* graph1, Graph_Data* graph2)
{
    UI_Packhead head;
    UI_Data_Operate operate;

    //帧头内容初始化
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 2;        //应该为6+30
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //机器人间通讯包头初始化
    operate.Data_ID = 0x102;                //自定义ID
    operate.Sender_ID = get_robot_id();     //发送端ID
    operate.Receiver_ID = get_client_id();  //接收端ID

    //数据转存
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), graph1, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data), graph2, sizeof(Graph_Data));

    //CRC校验
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 2 + 2);

    //DMA发送
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 2 + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}


/**
 *  @brief          使用自定义UI绘制五个图形，刷新出来在屏幕上显示
 *  @param[in]      图形1
 *  @param[in]      图形2
 *  @param[in]      图形3
 *  @param[in]      图形4
 *  @param[in]      图形5
 *  @return         none
 */
void ui_display_5_graph(Graph_Data* graph1, Graph_Data* graph2, Graph_Data* graph3, Graph_Data* graph4, Graph_Data* graph5)
{
    UI_Packhead head;
    UI_Data_Operate operate;

    //1. 帧头内容初始化
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 5;        //应该为6+75
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //2. 机器人间通讯包头初始化
    operate.Data_ID = 0x103;                //自定义ID，向客户端发送五个图形
    operate.Sender_ID = get_robot_id();     //发送端ID
    operate.Receiver_ID = get_client_id();  //接收端ID

    //3, 数据转存
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), graph1, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data), graph2, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 2, graph3, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 3, graph4, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 4, graph5, sizeof(Graph_Data));

    //4. CRC校验
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 5 + 2);

    //5. DMA发送
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 5 + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}


/**
 *  @brief          使用自定义UI绘制七个图形，刷新出来在屏幕上显示
 *  @param[in]      图形1
 *  @param[in]      图形2
 *  @param[in]      图形3
 *  @param[in]      图形4
 *  @param[in]      图形5
 *  @param[in]      图形6
 *  @param[in]      图形7
 *  @return         none
 */
void ui_display_7_graph(Graph_Data* graph1, Graph_Data* graph2, Graph_Data* graph3, Graph_Data* graph4, Graph_Data* graph5, Graph_Data* graph6, Graph_Data* graph7)
{
    UI_Packhead head;
    UI_Data_Operate operate;

    //1. 帧头内容初始化
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 7;        //应该为6+105
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //2. 机器人间通讯包头初始化
    operate.Data_ID = 0x104;                //自定义ID，向客户端发送七个图形
    operate.Sender_ID = get_robot_id();     //发送端ID
    operate.Receiver_ID = get_client_id();  //接收端ID

    //3, 数据转存
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), graph1, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data), graph2, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 2, graph3, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 3, graph4, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 4, graph5, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 5, graph6, sizeof(Graph_Data));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 6, graph7, sizeof(Graph_Data));

    //4. CRC校验
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 7 + 2);
  
    //5. DMA发送
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(Graph_Data) * 7 + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}


/**
 *  @brief          使用自定义UI绘制字符串
 *  @param[in]      字符串数据结构体指针
 *  @return         none
 */
void ui_display_string(String_Data* str_data)
{
    UI_Packhead head;
    UI_Data_Operate operate;

    //1. 帧头内容初始化
    head.SOF = 0xA5;
    head.Data_Length = sizeof(UI_Data_Operate) + sizeof(String_Data);  //应该为6+45
    head.Seq = UI_Seq++;
    head.CMD_ID = 0x301;

    //2. 机器人间通讯包头初始化
    operate.Data_ID = 0x110;                //自定义ID,绘制字符串
    operate.Sender_ID = get_robot_id();     //发送端ID
    operate.Receiver_ID = get_client_id();  //接收端ID

    //3. 数据转存
    memcpy(referee_uart_tx_buf[referee_tx_fifo], &head, sizeof(UI_Packhead));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead), &operate, sizeof(UI_Data_Operate));
    memcpy(referee_uart_tx_buf[referee_tx_fifo] + sizeof(UI_Packhead) + sizeof(UI_Data_Operate), str_data, sizeof(String_Data));

    //4. CRC校验
    Append_CRC8_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], 5);
    Append_CRC16_Check_Sum(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(String_Data) + 2);

    //5. DMA发送
    while (get_usart6_tx_dma_busy_flag())
    {
        osDelay(1);
    }
    usart6_tx_dma_enable(referee_uart_tx_buf[referee_tx_fifo], sizeof(UI_Packhead) + sizeof(UI_Data_Operate) + sizeof(String_Data) + 2);
    referee_tx_fifo = referee_tx_fifo == 0 ? 1 : 0;
}







uint8_t get_usart6_tx_dma_busy_flag(void)
{
    return usart6_tx_dma_is_busy;
    //return hdma_usart6_tx.State;
}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    usart6_tx_dma_is_busy = 1;

    HAL_UART_Transmit_DMA(&huart6, data, len);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart6)
    {
        clear_usart6_tx_dma_busy_sign();
    }
}


void clear_usart6_tx_dma_busy_sign(void)
{
    usart6_tx_dma_is_busy = 0;
}



uint8_t get_robot_id(void)
{
    return Judge_Hero.robot_status.robot_id;
}

uint16_t get_client_id(void)
{
    if (Judge_Hero.robot_status.robot_id < 100)
    {
        return 0x100 + Judge_Hero.robot_status.robot_id;
    }
    else
    {
        return 0x164 + (Judge_Hero.robot_status.robot_id - 100);
    }
}




