#include "User.h"
#include "Drive_PWM.h"

/************************* 超声波模块参数配置 *************************/
/* * 物理背景知识：常温下声速约为 343m/s。
 * 1微秒(us)声波传播的距离 = 343 * 10^2 / 10^6 = 0.0343 厘米 = 0.343 毫米。
 * 因为超声波是往返传播，所以实际测距中 1us 对应的距离为 0.343 / 2 = 0.1715 毫米。
 */

// 测量超时时间(微秒)：对应最大测量距离约2米(往返时间=2m * 2 / 343m/s ≈ 11662us)
#define ULTRASONIC_TIMEOUT_US        12000U
// 最小有效回波时间(微秒)：过滤由于硬件电路串扰产生的极短干扰毛刺
#define ULTRASONIC_MIN_VALID_US      20U
// 接收盲区/消隐时间(微秒)：发射探头在发出声波后会有余震，接收电路在刚发射完的这段时间内会收到强烈的自身干扰。
// 设定220us盲区，意味着放弃测量极其贴近探头的距离（约合近距盲区3.7cm），防止自激误判。
#define ULTRASONIC_BLANKING_US       220U   
// 滤波采样次数：决定了单次有效测距需要采集多少个样本。必须配合去极值算法使用。
#define ULTRASONIC_FILTER_SAMPLES    9U
// 边沿缓存深度：用于保存在一个接收窗口内捕获到的完整波形边沿序列。
#define ULTRASONIC_EDGE_BUFFER_SIZE  32U
// 硬件比较器抖动/尖峰抑制阈值：太近的双边沿通常不是有效40kHz回波。
#define ULTRASONIC_EDGE_DEBOUNCE_US  4U
// 40kHz方波理论半周期约12.5us。由于模拟链路畸变，这里放宽容差窗口。
#define ULTRASONIC_BURST_HALF_PERIOD_MIN_US  8U
#define ULTRASONIC_BURST_HALF_PERIOD_MAX_US  18U
// 相邻两个半周期之和应接近完整周期25us，同样保留余量。
#define ULTRASONIC_BURST_FULL_PERIOD_MIN_US  20U
#define ULTRASONIC_BURST_FULL_PERIOD_MAX_US  32U
// 至少要看到连续5个完整周期(10个半周期)才认定不是前导杂波。
#define ULTRASONIC_BURST_MIN_HALF_CYCLES     10U
// 当边沿数量已足够多且后续进入静默区时，可提前结束等待。
#define ULTRASONIC_CAPTURE_EARLY_EDGE_COUNT  10U
#define ULTRASONIC_CAPTURE_QUIET_US          80U
// 增益补偿最大重试次数：当回波信号微弱未触发中断时，尝试提升可编程增益放大器(PGA)倍数的次数
#define ULTRASONIC_GAIN_RETRY_MAX    3U
#define ULTRASONIC_GAIN_VERIFY_DELAY_MS 500U
// 界面刷新比采样慢一些，减少屏幕抖动。
#define ULTRASONIC_MEASURE_UI_REFRESH_MS 250U
// Flash存储地址：STM32F407的Sector11起始地址。Sector11大小为128KB。
// 选择此区域主要是因为它位于Flash末尾，不易与用户的代码区(通常从0x08000000开始)发生冲突。
#define ULTRASONIC_FLASH_ADDR        0x080E0000U
// Flash数据魔数："USON"的ASCII码 (0x55 0x53 0x4F 0x4E)。
// 作用：单片机上电时，通过比对这个标志，判断Flash该区域存储的是否是有效且格式正确的校准数据，而非未初始化的乱码(0xFFFFFFFF)。
#define ULTRASONIC_FLASH_MAGIC       0x55534F4EU  
// Flash数据版本号：若后续更改了结构体大小或成员，可通过变更版本号使旧校准数据自动失效，防止内存越界或解析错误。
#define ULTRASONIC_FLASH_VERSION     0x00010002U  
#define ULTRASONIC_CALIB_POINT_COUNT 5U

/************************* 屏幕显示常量定义 *************************/
#define TITLE_STR        "超声波测距仪"         // 主界面标题
#define MODEL_VER_STR    "型号：HC-SR04"       // 硬件型号
#define USER_VER_STR     "版本：V1.0"          // 软件版本
#define MENU1_CHOICE1    "1. 实时测量"         // 菜单选项1
#define MENU1_CHOICE2    "2. 距离校准"         // 菜单选项2
#define MENU1_CHOICE3    "3. 系统状态"         // 菜单选项3
#define MENU1_CHOICE4    "4. 程控调节"         // 菜单选项4
#define MENU_CHOICE_NUM  4                    // 菜单选项总数

// 快速清屏用的空白字符串数组。直接利用覆盖写字符的方式来擦除上一帧显示的旧文本，比调用LCD清屏函数开销小。
#define UI_BLANK_TEXT_16 "                                                                "  // 16号字，64个空格
#define UI_BLANK_TEXT_24 "                                                "                // 24号字，48个空格
#define UI_BLANK_TEXT_32 "                                "                                // 32号字，32个空格
#define UI_VALUE_BLANK_24 "                        "                                       // 数值区域空白

/************************* 数据结构定义 *************************/
/**
 * @brief 超声波校准数据结构体
 * @note 用于分段线性插值校准。它将实际的非线性误差转化为多段直线来近似。
 * 结构体需要确保4字节对齐，因为Flash写入通常以Word(32位)为最小单元。
 */
typedef struct
{
    uint32_t magic;               // 数据合法性标志
    uint32_t version;             // 结构体版本号
    uint32_t point_us[ULTRASONIC_CALIB_POINT_COUNT];         // 存储5个标准距离下实际测得的回波时间(微秒)
    uint32_t reserved[1];         // 预留空间，保证结构体总大小为32字节，方便Flash操作扩展
} UltrasonicCalibData;

// 选定5个基准校准距离(单位：毫米)。最后一个锚点为1300mm。
static const uint16_t k_calib_distance_mm[ULTRASONIC_CALIB_POINT_COUNT] = {100, 300, 600, 900, 1300};
static const uint16_t k_compact_line_y[9] = {150U, 174U, 198U, 222U, 246U, 270U, 294U, 318U, 342U};

/************************* 全局变量定义 *************************/
/* * 带有 volatile 关键字的变量，代表它们会在外部中断 (EXTI) 中被随时修改。
 * 强制编译器每次用到它们时都去内存读取，防止因为编译器开启优化而导致状态机卡死。
 */
static volatile uint8_t g_measure_active = 0;   // 测量窗口开启标志(1:允许中断记录边沿)
static volatile uint16_t g_echo_edge_count = 0; // 当前接收窗口内已经记录的边沿数量
static volatile uint32_t g_echo_edge_time_us[ULTRASONIC_EDGE_BUFFER_SIZE] = {0};
static volatile uint8_t g_echo_edge_level[ULTRASONIC_EDGE_BUFFER_SIZE] = {0};

static uint8_t g_gain_settle_discard = 0;       // 运放增益切换后，硬件电路需要建立时间。此标志提示丢弃切换后的首次测量。
static uint32_t g_last_echo_us = 1500U;         // 保存上一次有效测量的回波时间。利用空间相关性，预测下一次所需的增益倍数。
static uint8_t g_ultrasonic_gain_code = PGA112_DEFAULT_GAIN_CODE; // 记录当前运放的增益挡位
static uint8_t g_last_burst_edge_count = 0;     // 上一次单次测量中记录到的有效边沿数
static uint8_t g_last_burst_half_cycles = 0;    // 上一次单次测量中匹配到的连续有效半周期数

// 校准相关状态变量
static UltrasonicCalibData g_calib = {0};       // 运行时加载到RAM的校准数据缓存
static uint8_t g_calib_valid = 0;               // 若Flash中数据不合法，此标志为0，系统降级使用理想公式计算

// UI状态机
static uint8_t g_menu_sign = 0;                 // 0=主菜单，1=测量，2=校准，3=状态，4=程控PWM测试

/************************* 函数声明 *************************/
// 系统初始化与主界面函数
static void Init_All(void);
static void Disp_Main(void);
static void Change_Menu(uint8_t menu_sign);

// UI底层组件与刷新相关函数
static void Clear_Work_Area(void);
static void Clear_Work_Text(void);
static void Draw_Work_Title(char *title);
static void Draw_Key_Tips(char *tip1, char *tip2);
static void Show_Text_Line(uint16_t line, char *text);
static void Show_Value_Line(uint16_t line, char *label, double value, char *format);
static void Show_Value_Only(uint16_t line, double value, char *format);
static void Show_Text_Value_Only(uint16_t line, char *text);
static void Show_Compact_Value_Only(uint8_t line, char *text);
static void Wait_Ps2KeyRelease(uint8_t key_value);

// 超声波驱动与测量核心函数
static void Ultrasonic_Timer_Init(void);
static void Ultrasonic_Echo_Init(void);
static void Ultrasonic_ApplyGain(uint8_t gain_code);
static uint8_t Ultrasonic_SelectGainCode(uint32_t echo_us);
static uint32_t Ultrasonic_EstimatePeakTime(uint32_t rise_us, uint32_t fall_us);
static uint8_t Ultrasonic_IsHalfPeriodValid(uint32_t delta_us);
static uint8_t Ultrasonic_IsFullPeriodValid(uint32_t delta_us);
static uint8_t Ultrasonic_ExtractValidBurst(uint32_t *echo_us);
static void Ultrasonic_PrepareGain(uint8_t retry_count);
static uint8_t Ultrasonic_MeasureOnce(uint32_t *echo_us);
static uint8_t Ultrasonic_MeasureFiltered(uint32_t *echo_us);
static void Sort_Samples(uint32_t *data, uint8_t length);

// Flash数据存储与数学校准算法
static void Calibration_Load(void);
static uint8_t Calibration_IsValid(const UltrasonicCalibData *calib);
static uint8_t Calibration_IsPointReasonable(const UltrasonicCalibData *calib, uint8_t step, uint32_t echo_us);
static uint8_t Calibration_Save(const UltrasonicCalibData *calib);
static float Convert_Time_To_Distance(uint32_t echo_us);
static float Convert_Time_To_Distance_Default(uint32_t echo_us);

// 菜单业务逻辑状态机处理器
static void MenuHandler_Measure(void);
static void MenuHandler_Calibrate(void);
static void MenuHandler_Status(void);
static void MenuHandler_PGA_Test(void);

void User_main(void)
{
    Init_All();          // 底层硬件初始化(GPIO、定时器、中断、读取Flash校准数据)
    Disp_Main();         // 绘制静态主框架

    // 采用死循环+状态机的裸机运行框架
    while(1)
    {
        switch(g_menu_sign)
        {
            case 0:  // 停留在主菜单：轮询按键检测
                if(Ps2KeyValue >= KeyValue_1 && Ps2KeyValue <= KeyValue_4)
                {
                    // 动态更新选项指示符 '>' 并切换状态
                    Change_Menu((uint8_t)(Ps2KeyValue - KeyValue_0));
                }
                break;
            case 1:  // 进入实时测量阻塞态
                MenuHandler_Measure();
                break;
            case 2:  // 进入多点校准阻塞态
                MenuHandler_Calibrate();
                break;
            case 3:  // 进入查看状态阻塞态
                MenuHandler_Status();
                break;
            case 4:  // 进入手动增益/PWM测试阻塞态
                MenuHandler_PGA_Test();
                break;
            default: // 防御性编程：状态跑飞时强制复位到主页
                g_menu_sign = 0;
                break;
        }
        delay_ms(10);  // 适当的休眠，避免按键轮询空耗过多能耗
    }
}

/************************* 系统初始化函数 *************************/
static void Init_All(void)
{
    LCD_Clear(Black);                  // 刷黑屏
    Ultrasonic_PWM_Init();             // 配置发射探头需要的40kHz方波触发环境
    Ultrasonic_Timer_Init();           // 配置TIM5用作高精度的纳秒/微秒级秒表
    Ultrasonic_Echo_Init();            // 配置外部引脚用来接收回波并触发中断
    Calibration_Load();                // 从Flash唤醒时恢复先前的校准数据
    
    // 初始化程控放大器至默认倍数
    Ultrasonic_ApplyGain(PGA112_DEFAULT_GAIN_CODE);
    g_gain_settle_discard = 0;
}

/************************* 主界面显示函数 *************************/
static void Disp_Main(void)
{
    uint8_t count;
    
    // 渲染顶部标题区
    OS_String_Show(272, 16, 32, 1, TITLE_STR);
    
    // 绘制横竖分割线，规划出左侧导航和右侧数据展示区
    LCD_Appoint_Clear(0, 64, 800, 72, White);    // 顶部横线
    LCD_Appoint_Clear(0, 440, 800, 448, White);  // 底部横线
    LCD_Appoint_Clear(250, 72, 252, 440, White); // 垂直分隔线
    
    // 渲染底部状态栏
    OS_String_Show(32, 456, 16, 1, MODEL_VER_STR); 
    OS_String_Show(680, 456, 16, 1, USER_VER_STR); 
    
    // 初始化左侧菜单的未选中标记 "-"
    for(count = 1; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * count), 32, 1, "-");
    }
    
    // 渲染菜单文字
    OS_String_Show(60, 96, 32, 1, MENU1_CHOICE1);
    OS_String_Show(60, 160, 32, 1, MENU1_CHOICE2);
    OS_String_Show(60, 224, 32, 1, MENU1_CHOICE3);
    OS_String_Show(60, 288, 32, 1, MENU1_CHOICE4);
}

/**
 * @brief 动态切换菜单的UI逻辑
 */
static void Change_Menu(uint8_t menu_sign)
{
    uint8_t count;

    // 先擦除旧的所有指示箭头
    for(count = 1; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * count), 32, 1, "-");
    }

    // 更新新选中的选项箭头
    if(menu_sign >= 1 && menu_sign <= MENU_CHOICE_NUM)
    {
        OS_String_Show(32, (uint16_t)(32 + 64 * menu_sign), 32, 1, ">");
        g_menu_sign = menu_sign;
    }
    else
    {
        g_menu_sign = 0;  // 按键逻辑错误保护
        Clear_Work_Area();
    }

    Ps2KeyValue = KeyValue_Null;  // 消费掉该次按键事件，防止连续触发
}

/************************* UI工具函数(界面刷新与文本渲染) *************************/
static void Clear_Work_Area(void) { Clear_Work_Text(); }

static void Clear_Work_Text(void)
{
    LCD_Appoint_Clear(260, 80, 798, 438, Black);
}

static void Draw_Work_Title(char *title) { OS_String_Show(280, 88, 32, 1, title); }
static void Draw_Key_Tips(char *tip1, char *tip2) { OS_String_Show(280, 400, 16, 1, tip1); OS_String_Show(280, 420, 16, 1, tip2); }
static void Show_Text_Line(uint16_t line, char *text) { uint16_t y = (uint16_t)(150 + line * 30); OS_String_Show(280, y, 24, 1, text); }
static void Show_Value_Line(uint16_t line, char *label, double value, char *format) { char temp[24]; uint16_t y = (uint16_t)(150 + line * 30); sprintf(temp, format, value); OS_String_Show(280, y, 24, 1, label); OS_String_Show(500, y, 24, 1, temp); }
static void Show_Value_Only(uint16_t line, double value, char *format) { char temp[24]; uint16_t y = (uint16_t)(150 + line * 30); sprintf(temp, format, value); OS_String_Show(500, y, 24, 1, temp); }
static void Show_Text_Value_Only(uint16_t line, char *text) { uint16_t y = (uint16_t)(150 + line * 30); OS_String_Show(500, y, 24, 1, text); }
static void Show_Compact_Value_Only(uint8_t line, char *text)
{
    if(line < (sizeof(k_compact_line_y) / sizeof(k_compact_line_y[0])))
    {
        OS_String_Show(500, k_compact_line_y[line], 24, 1, UI_VALUE_BLANK_24);
        OS_String_Show(500, k_compact_line_y[line], 24, 1, text);
    }
}

static void Wait_Ps2KeyRelease(uint8_t key_value)
{
    do
    {
        Ps2KeyValue = KeyValue_Null;
        delay_ms(30);
    } while(Ps2KeyValue == key_value);
}


/************************* 超声波硬件驱动核心层 *************************/
/**
 * @brief 初始化定时器5，用于精确计时回波时间
 * @note 为什么选TIM5：它是STM32中少有的32位定时器，溢出周期极长，不会像16位定时器那样在测量远距离时发生溢出翻转。
 */
static void Ultrasonic_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseStructInit(&tim_base);
    // APB1总线频率一般为 84MHz。预分频设为(84-1)，得到进入计数器的时钟为 1MHz。
    // 即定时器每计一个数，严格代表流逝了 1微秒(us)。
    tim_base.TIM_Prescaler = 84 - 1;                     
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;       // 递增计数模式
    tim_base.TIM_Period = 0xFFFFFFFFU;                   // MAX=0xFFFFFFFF，约等于4294秒才会溢出
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;           
    TIM_TimeBaseInit(TIM5, &tim_base);
    
    TIM_Cmd(TIM5, ENABLE);  // 定时器挂载后保持常开，我们在中断中读取计数值
}

/**
 * @brief 初始化超声波回波引脚和外部中断
 * @note 为什么配置双边沿触发：接收到的超声波回波经过检波比较电路后，会输出一个脉冲信号。
 * 我们需要知道这个脉冲的高电平中心位置(代表真实峰值)，所以要同时抓取上升沿和下降沿。
 */
static void Ultrasonic_Echo_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    EXTI_InitTypeDef exti_init;
    NVIC_InitTypeDef nvic_init;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // PC0配置为浮空输入，依靠外接电路或传感器模块自带的推挽/开漏输出驱动
    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = GPIO_Pin_0;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio_init);

    // 将PC0映射到外部中断线EXTI0
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);

    EXTI_StructInit(&exti_init);
    exti_init.EXTI_Line = EXTI_Line0;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  // 关键：同时配置上升沿和下降沿触发
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);
    EXTI_ClearITPendingBit(EXTI_Line0);

    nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 2;  // 抢占优先级不能太低，以免由于其他中断延误导致测距出现微秒级误差
    nvic_init.NVIC_IRQChannelSubPriority = 1;        
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

/**
 * @brief 动态设置模拟前端放大器(PGA112)的增益倍数
 * @param gain_code 增益挡位编码
 */
static void Ultrasonic_ApplyGain(uint8_t gain_code)
{
    gain_code &= 0x07U;  // 掩码保护，防止非法越界

    // 增益每次只允许变动一档，避免从低倍数直接跳到高倍数。
    if(gain_code > g_ultrasonic_gain_code)
    {
        gain_code = (uint8_t)(g_ultrasonic_gain_code + 1U);
    }
    else if(gain_code < g_ultrasonic_gain_code)
    {
        gain_code = (uint8_t)(g_ultrasonic_gain_code - 1U);
    }

    if(gain_code != g_ultrasonic_gain_code)
    {
        PGA112_SetGainCode(gain_code);
        g_ultrasonic_gain_code = gain_code;
        // 增益切换后给模拟链路0.5s观察/校验窗口，再允许下一次继续变换。
        g_gain_settle_discard = 1;
        delay_ms(ULTRASONIC_GAIN_VERIFY_DELAY_MS);
    }
}

/**
 * @brief 距离与声场衰减预估策略
 * @note 声音在空气中传播遵循指数衰减和反平方根定律。距离越远，返回的信号越弱。
 * 这里根据历史回波时间(代表距离)，预测本次需要的增益倍数，避免远距离回波太弱检测不到。
 */
static uint8_t Ultrasonic_SelectGainCode(uint32_t echo_us)
{
    if(echo_us < 180U)       return PGA112_GAIN_1;    // 极近：不放大
    if(echo_us < 360U)       return PGA112_GAIN_2;    
    if(echo_us < 800U)       return PGA112_GAIN_4;    
    if(echo_us < 1800U)      return PGA112_GAIN_8;    
    if(echo_us < 3600U)      return PGA112_GAIN_16;   // 远距离：开始大幅度放大
    if(echo_us < 6500U)      return PGA112_GAIN_32;   
    if(echo_us < 9500U)      return PGA112_GAIN_64;   
    return PGA112_GAIN_128;                           // 极限距离：火力全开128倍
}

/**
 * @brief 估算真实回波的波峰时间
 * @note 回波电路输出的通常是一个有宽度的方波。
 * 回波最强点(波峰)其实出现在方波的中心位置。所以实际时间是：上升沿时间 + 脉宽的一半。
 */
static uint32_t Ultrasonic_EstimatePeakTime(uint32_t rise_us, uint32_t fall_us)
{
    uint32_t width;

    // 容错处理：如果下降沿时间小于上升沿(说明计数器翻转或时序错乱)，直接返回上升沿时间
    if(fall_us <= rise_us)
    {
        return rise_us;
    }

    width = fall_us - rise_us;
    return rise_us + width / 2U; // 中心点定位
}

static uint8_t Ultrasonic_IsHalfPeriodValid(uint32_t delta_us)
{
    return (uint8_t)(delta_us >= ULTRASONIC_BURST_HALF_PERIOD_MIN_US &&
                     delta_us <= ULTRASONIC_BURST_HALF_PERIOD_MAX_US);
}

static uint8_t Ultrasonic_IsFullPeriodValid(uint32_t delta_us)
{
    return (uint8_t)(delta_us >= ULTRASONIC_BURST_FULL_PERIOD_MIN_US &&
                     delta_us <= ULTRASONIC_BURST_FULL_PERIOD_MAX_US);
}

/**
 * @brief 从本轮接收窗口记录到的一串边沿中，提取连续稳定的40kHz波列
 * @note 这一步是“单次波形真假判别”的核心：如果前面只有1个周期杂波，后面才是真正的回波，
 *       那么算法会自动跳过短杂波，优先选择持续时间更长、更符合40kHz规律的边沿簇。
 */
static uint8_t Ultrasonic_ExtractValidBurst(uint32_t *echo_us)
{
    uint32_t edge_time[ULTRASONIC_EDGE_BUFFER_SIZE];
    uint8_t edge_level[ULTRASONIC_EDGE_BUFFER_SIZE];
    uint16_t raw_count;
    uint8_t clean_count = 0U;
    uint8_t best_start = 0U;
    uint8_t best_len = 0U;
    uint8_t start;
    uint8_t index;

    raw_count = g_echo_edge_count;
    if(raw_count > ULTRASONIC_EDGE_BUFFER_SIZE)
    {
        raw_count = ULTRASONIC_EDGE_BUFFER_SIZE;
    }

    // 先做一遍时间单调与最小间隔清洗，去掉比较器抖动产生的伪边沿
    for(index = 0U; index < raw_count; index++)
    {
        uint32_t now = g_echo_edge_time_us[index];
        uint8_t level = g_echo_edge_level[index];

        if(clean_count > 0U)
        {
            uint32_t prev = edge_time[clean_count - 1U];
            if(now <= prev)
            {
                continue;
            }
            if((now - prev) < ULTRASONIC_EDGE_DEBOUNCE_US)
            {
                continue;
            }
        }

        edge_time[clean_count] = now;
        edge_level[clean_count] = level;
        clean_count++;
    }

    g_last_burst_edge_count = clean_count;
    g_last_burst_half_cycles = 0U;

    if(clean_count < (ULTRASONIC_BURST_MIN_HALF_CYCLES + 1U))
    {
        return 0U;
    }

    // 扫描最长的“连续有效半周期”链。长度越长，越接近真正的8周期回波。
    for(start = 0U; start < (uint8_t)(clean_count - 1U); start++)
    {
        uint8_t run_len = 0U;
        uint32_t prev_delta = 0U;

        for(index = start; index < (uint8_t)(clean_count - 1U); index++)
        {
            uint32_t delta = edge_time[index + 1U] - edge_time[index];

            if(Ultrasonic_IsHalfPeriodValid(delta) == 0U)
            {
                break;
            }
            if(run_len > 0U && Ultrasonic_IsFullPeriodValid(prev_delta + delta) == 0U)
            {
                break;
            }

            prev_delta = delta;
            run_len++;
        }

        if(run_len > best_len)
        {
            best_len = run_len;
            best_start = start;
        }
    }

    g_last_burst_half_cycles = best_len;

    if(best_len < ULTRASONIC_BURST_MIN_HALF_CYCLES)
    {
        return 0U;
    }

    // 优先找该有效波列中的第一个“上升沿->下降沿”高电平脉冲中心，尽量保持与旧标定逻辑一致。
    for(index = best_start; index < (uint8_t)(best_start + best_len); index++)
    {
        if(edge_level[index] != 0U && edge_level[index + 1U] == 0U)
        {
            uint32_t width = edge_time[index + 1U] - edge_time[index];
            if(Ultrasonic_IsHalfPeriodValid(width) != 0U)
            {
                *echo_us = Ultrasonic_EstimatePeakTime(edge_time[index], edge_time[index + 1U]);
                return 1U;
            }
        }
    }

    // 若波列从下降沿切入，则退化为取第一对有效相邻边沿的中心点。
    for(index = best_start; index < (uint8_t)(best_start + best_len); index++)
    {
        uint32_t width = edge_time[index + 1U] - edge_time[index];
        if(Ultrasonic_IsHalfPeriodValid(width) != 0U)
        {
            *echo_us = edge_time[index] + width / 2U;
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief 自适应增益准备
 * @param retry_count 连续测距失败后的补偿次数
 */
static void Ultrasonic_PrepareGain(uint8_t retry_count)
{
    // 基于上一次有效回波时间，推断基础增益档位
    uint8_t gain_code = Ultrasonic_SelectGainCode(g_last_echo_us);

    // 如果连续测量失败(没收到回波)，说明信号太弱，每次重试将增益强制抬高一个挡位
    if(retry_count > ULTRASONIC_GAIN_RETRY_MAX)
    {
        retry_count = ULTRASONIC_GAIN_RETRY_MAX;
    }

    gain_code = (uint8_t)(gain_code + retry_count);
    if(gain_code > PGA112_GAIN_128)
    {
        gain_code = PGA112_GAIN_128; // 防止溢出最大档位
    }

    Ultrasonic_ApplyGain(gain_code);
    delay_us(20); // 留给运放SPI通信和电路趋于稳定的时间
}

/**
 * @brief 物理层：执行单次完整的超声波发射与捕获流程
 * @return 1=捕捉到回波脉冲；0=超时未检测到
 */
static uint8_t Ultrasonic_MeasureOnce(uint32_t *echo_us)
{
    uint32_t timeout;

    // 1. 复位本轮边沿捕获状态
    g_measure_active = 1;
    g_echo_edge_count = 0U;
    g_last_burst_edge_count = 0U;
    g_last_burst_half_cycles = 0U;

    // 2. 清零微秒级秒表，清除陈旧的外部中断请求
    TIM_SetCounter(TIM5, 0);
    EXTI_ClearITPendingBit(EXTI_Line0);
    
    // 3. 驱动探头发射一阵超声波脉冲序列(如40kHz，8个周期)
    Ultrasonic_FireBurst();

    // 4. 等待回波(阻塞轮询模式)
    // 每次delay 10us，循环次数 = 超时总时间 / 10
    for(timeout = 0; timeout < ULTRASONIC_TIMEOUT_US / 10U; timeout++)
    {
        // 若已经捕获到足够多的边沿，且后续进入静默区，可提前结束本轮等待。
        if(g_echo_edge_count >= ULTRASONIC_CAPTURE_EARLY_EDGE_COUNT)
        {
            uint16_t edge_count = g_echo_edge_count;
            uint32_t last_edge = g_echo_edge_time_us[edge_count - 1U];
            uint32_t now = TIM_GetCounter(TIM5);

            if(now > last_edge && (now - last_edge) >= ULTRASONIC_CAPTURE_QUIET_US)
            {
                break;
            }
        }

        // 边沿缓存打满则直接进入波列分析，避免继续等待覆盖有效数据。
        if(g_echo_edge_count >= ULTRASONIC_EDGE_BUFFER_SIZE)
        {
            break;
        }
        delay_us(10);
    }

    // 5. 关闭测量窗口，并对整串边沿执行真假判别
    g_measure_active = 0;
    return Ultrasonic_ExtractValidBurst(echo_us);
}

/**
 * @brief 算法层：执行带滤波的超声波测量(去极值平滑平均法)
 * @note 为什么需要这步：空气乱流、硬件噪声、被测物体不平整极易导致单次读数出现离谱跳变。
 */
static uint8_t Ultrasonic_MeasureFiltered(uint32_t *echo_us)
{
    uint32_t samples[ULTRASONIC_FILTER_SAMPLES]; // 样本池
    uint8_t valid_count = 0;                     // 有效样本数
    uint8_t attempts = 0;                        // 发射尝试总次数
    uint8_t gain_retry = 0;                      // 失败重试抬升增益计步器
    uint32_t sum = 0;
    uint8_t index;

    // 在总尝试次数耗尽前，努力填满所需样本池
    while(attempts < (ULTRASONIC_FILTER_SAMPLES + 6U) && valid_count < ULTRASONIC_FILTER_SAMPLES)
    {
        uint32_t sample = 0;

        // 动态匹配增益
        Ultrasonic_PrepareGain(gain_retry); 
        attempts++;

        if(Ultrasonic_MeasureOnce(&sample) != 0U)
        {
            // 如果刚刚切换了增益，硬件处于震荡期，主动丢弃这帧虽然有效但可能畸变的脏数据
            if(g_gain_settle_discard != 0U)
            {
                g_gain_settle_discard = 0;
                delay_ms(20);
                continue;
            }

            // 成功采集一条有效数据存入池中
            samples[valid_count++] = sample;
            g_last_echo_us = sample; // 刷新测距历史记忆
            gain_retry = 0;          // 采集成功，重试增益计步清零
        }
        else
        {
            // 未收到回波处理逻辑
            if(g_gain_settle_discard != 0U)
            {
                g_gain_settle_discard = 0; // 同样丢弃增益不稳定期的失败
            }
            else if(gain_retry < ULTRASONIC_GAIN_RETRY_MAX)
            {
                gain_retry++; // 增加一级增益，下次尝试接收微弱回波
            }
        }
        // 降低探头发射占空比，防止声波在狭小空间反射堆积形成驻波干扰 
        delay_ms(20); 
    }

    if(valid_count == 0U)
    {
        return 0; // 彻底失败
    }

    // 将收集到的样本进行升序排序
    Sort_Samples(samples, valid_count);
    
    // 核心算法：掐头去尾求平均
    if(valid_count >= 3U)
    {
        // 如果样本充足(>=5个)，掐掉2个最大值和2个最小值；否则只掐掉最大最小各1个
        uint8_t trim = (valid_count >= 5U) ? 2U : 1U;
        
        for(index = trim; index < (uint8_t)(valid_count - trim); index++)
        {
            sum += samples[index];
        }
        // 计算抛弃极端值后的平均数
        *echo_us = sum / (valid_count - (uint8_t)(trim * 2U));
    }
    else
    {
        // 如果样本少于3个，直接取中间那个数
        *echo_us = samples[valid_count / 2U];
    }

    g_last_echo_us = *echo_us;
    return 1;
}

/**
 * @brief 使用插入排序算法对采样数组进行升序排列
 */
static void Sort_Samples(uint32_t *data, uint8_t length)
{
    uint8_t i;

    // 经典的插入排序算法
    for(i = 1U; i < length; i++)
    {
        uint32_t key = data[i];
        int8_t j = (int8_t)i - 1;

        while(j >= 0 && data[j] > key)
        {
            data[j + 1] = data[j];
            j--;
        }
        data[j + 1] = key;
    }
}

/************************* 校准与距离转换数据层 *************************/
/**
 * @brief 从Flash中唤醒并加载历史校准数据
 */
static void Calibration_Load(void)
{
    // 利用C语言指针特性，直接将Flash基地址强转为结构体指针读取
    const UltrasonicCalibData *stored = (const UltrasonicCalibData *)ULTRASONIC_FLASH_ADDR;
    g_calib = *stored; 
    g_calib_valid = Calibration_IsValid(&g_calib); 
}

/**
 * @brief 校验Flash中读取数据的合法性
 */
static uint8_t Calibration_IsValid(const UltrasonicCalibData *calib)
{
    uint8_t index;

    // 1. 魔法字和版本防呆拦截
    if(calib->magic != ULTRASONIC_FLASH_MAGIC || calib->version != ULTRASONIC_FLASH_VERSION)
    {
        return 0;
    }

    // 2. 逻辑一致性校验：物理世界中，距离越远，返回时间必然越长
    for(index = 0; index < ULTRASONIC_CALIB_POINT_COUNT; index++)
    {
        if(calib->point_us[index] == 0U || calib->point_us[index] > ULTRASONIC_TIMEOUT_US)
        {
            return 0; // 数据越界
        }
        if(index > 0U && calib->point_us[index] <= calib->point_us[index - 1U])
        {
            return 0; // 如果出现了后一个标定点的微秒数小于等于前一个点，说明记录错乱
        }
    }

    return 1;  
}

/**
 * @brief 判断当前采集到的校准点读数是否在合理的时间窗内
 */
static uint8_t Calibration_IsPointReasonable(const UltrasonicCalibData *calib, uint8_t step, uint32_t echo_us)
{
    uint32_t min_us;
    uint32_t max_us;
    uint32_t expected_us;

    if(step >= ULTRASONIC_CALIB_POINT_COUNT)
    {
        return 0U;
    }

    expected_us = (uint32_t)((float)k_calib_distance_mm[step] / 0.1715f);
    min_us = (expected_us > 500U) ? (expected_us - 500U) : ULTRASONIC_MIN_VALID_US;
    max_us = expected_us + 1800U;

    if(echo_us < min_us || echo_us > max_us)
    {
        return 0U;
    }

    if(step > 0U && echo_us <= calib->point_us[step - 1U])
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 将校准生成的结构体持久化写入Flash
 * @note Flash有寿命限制(约十万次擦写)，不能无节制调用该函数。
 */
static uint8_t Calibration_Save(const UltrasonicCalibData *calib)
{
    FLASH_Status status = FLASH_COMPLETE;
    const uint32_t *words = (const uint32_t *)calib; 
    uint32_t address = ULTRASONIC_FLASH_ADDR;
    uint32_t index;

    FLASH_Unlock(); // 开放对Flash控制寄存器的访问权限
    
    // 清除由于之前误操作遗留的错误标志位
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    
    // 必须先将Sector11整个扇区擦除(所有位变成1)，Flash只能将1写成0，不能反向写。
    status = FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
    
    if(status == FLASH_COMPLETE)
    {
        // 每次写入一个32位的数据(Word)，循环写入结构体所有空间
        for(index = 0; index < (sizeof(UltrasonicCalibData) / 4U); index++)
        {
            status = FLASH_ProgramWord(address, words[index]);
            if(status != FLASH_COMPLETE)
            {
                break; 
            }
            address += 4U;  // 内存地址递增4字节
        }
    }
    
    FLASH_Lock(); // 重新上锁保护代码区

    return (uint8_t)(status == FLASH_COMPLETE); 
}

/**
 * @brief 无校准数据时的理想物理公式回退方案
 */
static float Convert_Time_To_Distance_Default(uint32_t echo_us)
{
    // 理想公式：距离(mm) = 时间(us) * 0.1715
    float distance = (float)echo_us * 0.1715f;

    // 输出限幅滤波器：钳制在硬件合理量程区间
    if(distance < 10.0f) distance = 10.0f;
    if(distance > (float)k_calib_distance_mm[ULTRASONIC_CALIB_POINT_COUNT - 1U]) distance = (float)k_calib_distance_mm[ULTRASONIC_CALIB_POINT_COUNT - 1U];
    return distance;
}

/**
 * @brief 基于用户标定数据的分段线性插值距离结算
 * @note 将真实测距过程产生的硬件系统延迟、声速温度漂移等复合误差，
 * 通过建立5个锚点的4段折线函数，近似映射为线性关系。
 */
static float Convert_Time_To_Distance(uint32_t echo_us)
{
    uint8_t index;
    float x0, x1;  // 插值区间左、右边界的微秒时间(us)
    float y0, y1;  // 插值区间左、右边界的标准距离(mm)
    float distance;

    if(g_calib_valid == 0U)
    {
        return Convert_Time_To_Distance_Default(echo_us);
    }

    // 寻找当前回波时间所在的插值区间
    index = 0U;
    while(index < (ULTRASONIC_CALIB_POINT_COUNT - 2U) && echo_us > g_calib.point_us[index + 1U])
    {
        index++;
    }

    x0 = (float)g_calib.point_us[index];
    x1 = (float)g_calib.point_us[index + 1U];
    y0 = (float)k_calib_distance_mm[index];
    y1 = (float)k_calib_distance_mm[index + 1U];

    if(x1 <= x0) // 防御性编程：防止分母为0
    {
        return Convert_Time_To_Distance_Default(echo_us);
    }

    /* * 分段线性插值公式计算距离 y 
     * y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
     */
    distance = y0 + ((float)echo_us - x0) * (y1 - y0) / (x1 - x0);
    
    // 输出限幅
    if(distance < 10.0f) distance = 10.0f;
    if(distance > (float)k_calib_distance_mm[ULTRASONIC_CALIB_POINT_COUNT - 1U]) distance = (float)k_calib_distance_mm[ULTRASONIC_CALIB_POINT_COUNT - 1U];
    return distance;
}

/************************* 菜单状态机处理器(业务层) *************************/
static void MenuHandler_Measure(void)
{
    char value_text[24];

    Draw_Work_Title("测量模式");
    Draw_Key_Tips("确认开始测量", "返回退出测量");
    OS_String_Show(280, 150, 24, 1, "测量时间(us)");
    OS_String_Show(280, 180, 24, 1, "测量距离(mm)");
    OS_String_Show(280, 210, 24, 1, "默认距离(mm)");
    OS_String_Show(280, 240, 24, 1, "校准状态");
    OS_String_Show(280, 270, 24, 1, "前端增益(x)");
    OS_String_Show(280, 300, 24, 1, "提示信息");
    Show_Text_Value_Only(3, "未校准");
    Show_Text_Value_Only(4, "008");
    Show_Text_Value_Only(5, "等待开始");

    while(Ps2KeyValue != KeyValue_Back)
    {
        uint32_t echo_us = 0;
        if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
        {
            float distance = Convert_Time_To_Distance(echo_us);
            sprintf(value_text, "%05lu", (unsigned long)echo_us);
            Show_Text_Value_Only(0, value_text);
            sprintf(value_text, "%06.1f", (double)distance);
            Show_Text_Value_Only(1, value_text);
            sprintf(value_text, "%06.1f", (double)Convert_Time_To_Distance_Default(echo_us));
            Show_Text_Value_Only(2, value_text);
            Show_Text_Value_Only(3, (g_calib_valid != 0U) ? "已校准  " : "未校准  ");
            sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
            Show_Text_Value_Only(4, value_text);
            sprintf(value_text, "有效%02u周", (unsigned int)(g_last_burst_half_cycles / 2U));
            Show_Text_Value_Only(5, value_text);
        }
        else
        {
            Show_Text_Value_Only(3, "测量失败");
            sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
            Show_Text_Value_Only(4, value_text);
            Show_Text_Value_Only(5, (g_last_burst_edge_count > 0U) ? "疑似杂波" : "检查探头");
        }
        
        delay_ms(ULTRASONIC_MEASURE_UI_REFRESH_MS);
    }

    Ps2KeyValue = KeyValue_Null;
    Change_Menu(0);
}

static void MenuHandler_Calibrate(void)
{
    // 实例化一个临时校准结构体用作数据收集
    UltrasonicCalibData new_calib;
    uint8_t index;
    uint8_t step = 0;
    uint8_t last_step = 0xFFU;
    char line[24];

    Clear_Work_Area();

    // 初始化内存结构体
    new_calib.magic = ULTRASONIC_FLASH_MAGIC;
    new_calib.version = ULTRASONIC_FLASH_VERSION;
    for(index = 0; index < ULTRASONIC_CALIB_POINT_COUNT; index++)
    {
        new_calib.point_us[index] = 0U;
    }
    new_calib.reserved[0] = 0U;

    Draw_Work_Title("距离校准");
    Draw_Key_Tips("确定捕获当前点", "返回放弃校准");
    OS_String_Show(280, k_compact_line_y[0], 24, 1, "当前目标");
    OS_String_Show(280, k_compact_line_y[1], 24, 1, "采集状态");
    OS_String_Show(280, k_compact_line_y[2], 24, 1, "100mm(us)");
    OS_String_Show(280, k_compact_line_y[3], 24, 1, "300mm(us)");
    OS_String_Show(280, k_compact_line_y[4], 24, 1, "600mm(us)");
    OS_String_Show(280, k_compact_line_y[5], 24, 1, "900mm(us)");
    OS_String_Show(280, k_compact_line_y[6], 24, 1, "1300mm(us)");
    OS_String_Show(280, k_compact_line_y[7], 24, 1, "????(us)");
    OS_String_Show(280, k_compact_line_y[8], 24, 1, "系统提示");
    Show_Compact_Value_Only(1, "等待开始");
    Show_Compact_Value_Only(2, "00000");
    Show_Compact_Value_Only(3, "00000");
    Show_Compact_Value_Only(4, "00000");
    Show_Compact_Value_Only(5, "00000");
    Show_Compact_Value_Only(6, "00000");
    Show_Compact_Value_Only(7, "00000");
    Show_Compact_Value_Only(8, "就绪");

    while(Ps2KeyValue != KeyValue_Back)
    {
        if(step != last_step)
        {
            sprintf(line, "请放至%04umm", k_calib_distance_mm[step]);
            Show_Compact_Value_Only(0, line);
            last_step = step;
        }

        if(Ps2KeyValue == KeyValue_Enter)
        {
            uint32_t echo_us = 0;
            Ps2KeyValue = KeyValue_Null;

            Show_Compact_Value_Only(0, "正在测量 ");
            Show_Compact_Value_Only(1, "计算中 ");
            if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
            {
                if(Calibration_IsPointReasonable(&new_calib, step, echo_us) == 0U)
                {
                    sprintf(line, "%05lu", (unsigned long)echo_us);
                    Show_Compact_Value_Only(7, line);
                    Show_Compact_Value_Only(1, "数据异常 ");
                    Show_Compact_Value_Only(8, "超出合理误差区间 ");
                    Wait_Ps2KeyRelease(KeyValue_Enter);
                    delay_ms(20);
                    continue;
                }

                new_calib.point_us[step] = echo_us;
                sprintf(line, "%05lu", (unsigned long)echo_us);
                Show_Compact_Value_Only(7, line);
                Show_Compact_Value_Only((uint8_t)(2U + step), line);
                Show_Compact_Value_Only(1, "捕获成功 ");

                step++;
                if(step >= ULTRASONIC_CALIB_POINT_COUNT)
                {
                    if(Calibration_IsValid(&new_calib) != 0U && Calibration_Save(&new_calib) != 0U)
                    {
                        g_calib = new_calib;
                        g_calib_valid = 1;
                        Show_Compact_Value_Only(8, "保存成功 ");
                    }
                    else
                    {
                        Show_Compact_Value_Only(8, "写入失败 ");
                    }
                    delay_ms(1000);
                    break;
                }
            }
            else
            {
                Show_Compact_Value_Only(1, "测量失败 ");
                Show_Compact_Value_Only(8, "未捕获到回波 ");
            }
            Wait_Ps2KeyRelease(KeyValue_Enter);
        }
        delay_ms(20);
    }

    Ps2KeyValue = KeyValue_Null;
    Change_Menu(0);
}

static void MenuHandler_Status(void)
{
    char value_text[24];

    Clear_Work_Area();

    Draw_Work_Title("系统状态");
    Draw_Key_Tips("确定单次测试", "返回主页菜单");
    OS_String_Show(280, k_compact_line_y[0], 24, 1, "存储状态");
    OS_String_Show(280, k_compact_line_y[1], 24, 1, "锚点1(us)");
    OS_String_Show(280, k_compact_line_y[2], 24, 1, "锚点2(us)");
    OS_String_Show(280, k_compact_line_y[3], 24, 1, "锚点3(us)");
    OS_String_Show(280, k_compact_line_y[4], 24, 1, "锚点4(us)");
    OS_String_Show(280, k_compact_line_y[5], 24, 1, "锚点5(us)");
    OS_String_Show(280, k_compact_line_y[6], 24, 1, "系统当前增益(x)");
    OS_String_Show(280, k_compact_line_y[7], 24, 1, "测试回波时间(us)");
    OS_String_Show(280, k_compact_line_y[8], 24, 1, "动态解析距离(mm)");

    Show_Compact_Value_Only(0, (g_calib_valid != 0U) ? "有效校准数据" : "默认理想数据");
    sprintf(value_text, "%05lu", (unsigned long)g_calib.point_us[0]); Show_Compact_Value_Only(1, value_text);
    sprintf(value_text, "%05lu", (unsigned long)g_calib.point_us[1]); Show_Compact_Value_Only(2, value_text);
    sprintf(value_text, "%05lu", (unsigned long)g_calib.point_us[2]); Show_Compact_Value_Only(3, value_text);
    sprintf(value_text, "%05lu", (unsigned long)g_calib.point_us[3]); Show_Compact_Value_Only(4, value_text);
    sprintf(value_text, "%05lu", (unsigned long)g_calib.point_us[4]); Show_Compact_Value_Only(5, value_text);
    sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code)); Show_Compact_Value_Only(6, value_text);
    Show_Compact_Value_Only(7, "00000");
    Show_Compact_Value_Only(8, "0000.0");

    while(Ps2KeyValue != KeyValue_Back)
    {
        if(Ps2KeyValue == KeyValue_Enter)
        {
            uint32_t echo_us = 0;
            Ps2KeyValue = KeyValue_Null;
            if(Ultrasonic_MeasureFiltered(&echo_us) != 0U)
            {
                sprintf(value_text, "%05lu", (unsigned long)echo_us);
                Show_Compact_Value_Only(7, value_text);
                sprintf(value_text, "%06.1f", (double)Convert_Time_To_Distance(echo_us));
                Show_Compact_Value_Only(8, value_text);
            }
            else
            {
                Show_Compact_Value_Only(7, "超时 ");
                Show_Compact_Value_Only(8, "0000.0");
            }
        }
        delay_ms(20);
    }

    Ps2KeyValue = KeyValue_Null;
    Change_Menu(0);
}

/**
 * @brief 菜单状态机处理器：PGA 与 PWM 联调测试模式
 * @note 允许用户在开启 40kHz 互补 PWM 发射的同时，手动调节 PGA112 的增益，用于硬调测试。
 */
static void MenuHandler_PGA_Test(void)
{
    uint8_t gain_index = g_ultrasonic_gain_code; // 从当前真实挡位起步，避免界面值和硬件值脱节
    const uint8_t gain_codes[8] =
    {
        PGA112_GAIN_1,  PGA112_GAIN_2,  PGA112_GAIN_4,  PGA112_GAIN_8,
        PGA112_GAIN_16, PGA112_GAIN_32, PGA112_GAIN_64, PGA112_GAIN_128
    };
    char value_text[24];

    // 激活物理层发射并写入初始增益
    Ultrasonic_PWM_OutputEnable();
    Ultrasonic_ApplyGain(gain_codes[gain_index]);

    // 静态文本渲染
    Draw_Work_Title("程控增益调节");
    Draw_Key_Tips("+/-调节增益", "Back退出并关闭PWM");
    OS_String_Show(280, 150, 24, 1, "PWM输出状态");
    OS_String_Show(280, 180, 24, 1, "输出方式");
    OS_String_Show(280, 210, 24, 1, "输出频率(Hz)");
    OS_String_Show(280, 240, 24, 1, "输出占空比(%)");
    OS_String_Show(280, 270, 24, 1, "当前增益(x)");
    OS_String_Show(280, 300, 24, 1, "增益档位");
    OS_String_Show(280, 330, 24, 1, "波形说明");
    OS_String_Show(280, 360, 24, 1, "当前提示");

    // 初始参数渲染
    Show_Text_Value_Only(0, "开启");
    Show_Text_Value_Only(1, "互补PWM");
    Show_Text_Value_Only(2, "040000"); // 40kHz 探头谐振频率
    Show_Text_Value_Only(3, "050");    // 50% 占空比
    sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
    Show_Text_Value_Only(4, value_text);
    Show_Text_Value_Only(5, "1/2/4/8/16/32/64/128");
    Show_Text_Value_Only(6, "PD12/PD13输出");
    Show_Text_Value_Only(7, "等待调节");

    // 动态事件轮询
    while(Ps2KeyValue != KeyValue_Back)
    {
        if(Ps2KeyValue == KeyValue_Add)
        {
            Ps2KeyValue = KeyValue_Null; // 及时消费事件
            if(gain_index < 7U)
            {
                gain_index++;
                Ultrasonic_ApplyGain(gain_codes[gain_index]);
                sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
                Show_Text_Value_Only(4, value_text);
                Show_Text_Value_Only(7, "增益已调大");
            }
            else
            {
                Show_Text_Value_Only(7, "已到最大增益");
            }
            Wait_Ps2KeyRelease(KeyValue_Add); // 增加按键释放等待，防止死循环快速累加
        }
        else if(Ps2KeyValue == KeyValue_Minus)
        {
            Ps2KeyValue = KeyValue_Null; // 及时消费事件
            if(gain_index > 0U)
            {
                gain_index--;
                Ultrasonic_ApplyGain(gain_codes[gain_index]);
                sprintf(value_text, "%03u", PGA112_GetGainValue(g_ultrasonic_gain_code));
                Show_Text_Value_Only(4, value_text);
                Show_Text_Value_Only(7, "增益已调小");
            }
            else
            {
                Show_Text_Value_Only(7, "已到最小增益");
            }
            Wait_Ps2KeyRelease(KeyValue_Minus); // 增加按键释放等待
        }
        delay_ms(20);
    }

    // 安全关闭：退出时务必关闭物理驱动，防止探头持续高压发热
    Ultrasonic_PWM_OutputDisable();
    Ps2KeyValue = KeyValue_Null;
    Change_Menu(0);
}

/************************* 中断服务函数 *************************/

/**
 * @brief EXTI0外部中断服务函数 (回波信号捕捉状态机)
 * @note 当回波引脚(PC0)电平反转时，触发该中断。
 * 状态机流转：过滤盲区 -> 捕捉上升沿 -> 捕捉下降沿 -> 换算中心点时间
 */
void EXTI0_IRQHandler(void)
{
    // 检查是否是由于Line0引发的中断
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        // 只有软件开启了允许测量窗口，才对外部引脚波动进行响应，避免杂波干扰系统
        if(g_measure_active != 0U)
        {
            uint32_t now = TIM_GetCounter(TIM5); // 获取触发瞬间的系统微秒数

            // 状态机阶段 1：过滤硬件发射时由于自身耦合产生的极近距离强回波盲区(例如220us)
            if(now >= ULTRASONIC_BLANKING_US)
            {
                uint16_t edge_index = g_echo_edge_count;

                if(edge_index < ULTRASONIC_EDGE_BUFFER_SIZE)
                {
                    uint8_t level = (uint8_t)(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) != Bit_RESET);

                    // 在中断里先做一次最小间隔抑制，降低抖动边沿污染。
                    if(edge_index == 0U ||
                       (now > g_echo_edge_time_us[edge_index - 1U] &&
                        (now - g_echo_edge_time_us[edge_index - 1U]) >= ULTRASONIC_EDGE_DEBOUNCE_US))
                    {
                        g_echo_edge_time_us[edge_index] = now;
                        g_echo_edge_level[edge_index] = level;
                        g_echo_edge_count = (uint16_t)(edge_index + 1U);
                    }
                }
                else
                {
                    g_measure_active = 0U;
                }
            }
        }
        // 规范：清除中断挂起标志位必须放在标准中断处理末尾
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
