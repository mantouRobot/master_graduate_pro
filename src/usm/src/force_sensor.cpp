// 力传感器

#include <usm/force_sensor.h>

namespace main_class{

#define FORCE_BUFFER_SIZE 16384	//Ïà¹ØµÄ»ºŽæŽóÐ¡
unsigned char * g_ForceReadBuffer = new unsigned char[FORCE_BUFFER_SIZE];
unsigned char * g_ForceMyRxBuffer = new unsigned char[FORCE_BUFFER_SIZE];	//
uint64_t	g_MyRxBufferCounter = 0;
uint64_t	g_DataPtrOut = 0;
volatile double	g_ForceBuffer[6] = {0.0};
volatile uint64_t	g_ForceIndex = 0;
const double m_dDecouplingCoefficient[6][6]=
{
  {0.190660744, 	0.223685906, 	0.534145445, 	18.003595500 ,	0.078319299, 	-17.726435630},
  {0.107527388 ,	-20.122044610 ,	0.477155640 ,	10.616147420 ,	0.295934085 ,	10.000624860},
  {56.159414210 ,	-0.330263321 ,	55.926562190 ,	1.128619676 ,	54.707066010 ,	-0.994680395},
  {-0.032886763, 	-0.013842118 ,	-0.308634583 ,	0.003593230 ,	0.251016213 ,	0.004968525 },
  {0.346667582 ,	0.000148175 ,	-0.131789607 ,	-0.016990855 ,	-0.143606749 ,	0.017654130 },
  {0.000069847 ,	0.125876500 ,	0.003589640 ,	0.118673608 ,	-0.000442646 ,	0.086879145 },
};

double m_dAmpZero[6] = { 32716.000000, 32764.000000, 32761.000000,32800.000000,32683.000000,32672.000000};
double m_dChnGain[6] = { 123.080090,	123.213244,	123.044366,	123.158034,	123.054109,	123.115814};
double m_dChnEx[6] = { 2.515955,	2.515955,	2.515955,	2.515955,	2.515955,	2.515955};

ForceSensor::ForceSensor(ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh),
  pnh_(nh),
  force_serial_("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000))
{
  debug_pub_ = nh_.advertise<std_msgs::Empty>("/force_debug", 1);
  init();
}

ForceSensor::~ForceSensor() {
  if(force_serial_.isOpen()) {
    force_serial_.flush();
    force_serial_.close();
  }
  if(force_thread_ != nullptr) {
    force_thread_->interrupt();
    force_thread_->join();
    force_thread_ = nullptr;
  }
}

void ForceSensor::init() {
  // 驱动器串口打开初始化
  if(!force_serial_.isOpen()) {
    force_serial_.open();
    if(!force_serial_.isOpen())
      ROS_ERROR("Motor serial open failed!");
  }
  ROS_INFO("Force serial init successed.");

  force_serial_.flush();

  ros::Rate r(20);
  // 波特率
  force_cmd_ = "AT+SMPR=500\r\n";
  force_serial_.write(force_cmd_);
  r.sleep();
  // 通道
  force_cmd_ = "AT+SGDM=(A01,A02,A03,A04,A05,A06);C;1;(WMA:1)\r\n";
  force_serial_.write(force_cmd_);
  r.sleep();
  // 开始传输
  force_cmd_ = "AT+GSD\r\n";
  force_serial_.write(force_cmd_);
  r.sleep();

  // 开始数据处理线程
  force_thread_ = new boost::thread(boost::bind(&ForceSensor::processThread, this));
}

void ForceSensor::processThread() {
  ros::Rate r(500);
  while(ros::ok()) {
    r.sleep();

//    ROS_INFO_THROTTLE(1, "%.2f", g_ForceBuffer[2] - bias_);

    uint64_t len = 20;
    uint8_t *str;
    std::string s;
    s = force_serial_.read(len);
//    s = force_serial_.readline(1000, "0xAA");

    uint64_t i = 0x00;
    memcpy(g_ForceReadBuffer, s.c_str(), len);

    for( i =0; i < len; i++)
    {
      if(g_MyRxBufferCounter >= FORCE_BUFFER_SIZE)
      {
        g_MyRxBufferCounter = 0;
      }
      g_ForceMyRxBuffer[g_MyRxBufferCounter++] =  g_ForceReadBuffer[i];//±£ŽæÔ­ÊŒÊýŸÝ£¬g_MyRxBufferCounter×îºóÖžÏòÊýŸÝµÄÏÂÒ»žöÎ»
    }

    processData(len);
  }
}

bool ForceSensor::processData(uint64_t len) {
  //***************ŽŠÀíÊýŸÝ******************
  //Head fram	  PackageLength	 DataNo	       Data	       ChkSum
  //0xAA,0x55	     HB,LB         2B   (ChNum*N*DNpCH) B	 1B		//(ChNum*N*DNpCH)Îª6*2*1=12B£¬ËùÒÔ¹²ÓÐ19×ÖœÚ
  //unsigned char g_ForceReadBuffer[FORCE_BUFFER_SIZE] = {0};	//³õÊŒ¶ÁÈ¡ÁŠŽ®¿ÚµÄÊý×é
  //volatile double	g_ForceBuffer[FORCE_BUFFER_SIZE][6] = {0.0};//Ë³ÐòÑ­»·±£ŽæFx/FY/FZ/MX/MY/MZ
  //volatile uint64_t	g_ForceIndex = 0;//µ±Ç°g_ForceBufferµÄË÷Òý

  if(g_ForceMyRxBuffer == nullptr) {return false;}
  if(g_DataPtrOut == 0xFFFFFFFF)	{return false;}

  uint64_t HeaderIndex =0x00;//ÖžÏò0x55µÄË÷Òý£¬ËùÓÐË÷Òý¶ŒŽÓ0¿ªÊŒ
  bool DataHeaderFlag = false;//Í·µÄ±êÖŸÎ»
  uint64_t PackageLength= 0xFFFFFFFF;
  uint64_t HighIndex,LowIndex;
  uint64_t RxLengthTemp = 0x00;
  uint64_t RxCounterCurrent = g_MyRxBufferCounter;
  uint64_t i = 0x00;

  while (1)
  {
    //--------------------------------------------------------------------------------
    //ÊýŸÝ³€¶È£¬ŽÓg_DataPtrOutËãÆð
    HeaderIndex = g_DataPtrOut;
    if(RxCounterCurrent >= HeaderIndex)
    {
      RxLengthTemp = RxCounterCurrent - HeaderIndex;
    }
    else
    {
      RxLengthTemp = FORCE_BUFFER_SIZE - HeaderIndex + RxCounterCurrent;
    }
    if(RxLengthTemp <= 9)
    {
      break;//ÍË³öwhile
    }
    //--------------------------------------------------------------------------------
    //Ñ°ÕÒÖ¡Í·
    for(i = 0x00;i < RxLengthTemp; i++)
    {
      if(HeaderIndex == FORCE_BUFFER_SIZE )
      {
        HeaderIndex = 0x00;
      }
      if(HeaderIndex == FORCE_BUFFER_SIZE-1)
      {
        if((g_ForceMyRxBuffer[HeaderIndex] == 0xAA) && (g_ForceMyRxBuffer[0] == 0x55))
        {
          DataHeaderFlag = true;
          break;
        }
      }
      else if((g_ForceMyRxBuffer[HeaderIndex] == 0xAA) && (g_ForceMyRxBuffer[HeaderIndex+1] == 0x55))
      {
        DataHeaderFlag = true;
        break;
      }
      HeaderIndex++;
    }
    if(DataHeaderFlag != true)
    {
      break;
    }
    DataHeaderFlag = false;
    //-------------------------------------------------------------------------------
    HighIndex = HeaderIndex+2;
    LowIndex = HeaderIndex+3;
    if(HighIndex >= FORCE_BUFFER_SIZE)
    {
      HighIndex = HighIndex - FORCE_BUFFER_SIZE;
    }
    if(LowIndex >= FORCE_BUFFER_SIZE)
    {
      LowIndex = LowIndex - FORCE_BUFFER_SIZE;
    }
    PackageLength = g_ForceMyRxBuffer[HighIndex]*256 + g_ForceMyRxBuffer[LowIndex];

    if(RxCounterCurrent >= HeaderIndex)
    {
      RxLengthTemp = RxCounterCurrent - HeaderIndex;
    }else
    {
      RxLengthTemp = FORCE_BUFFER_SIZE - HeaderIndex + RxCounterCurrent;
    }
    if(RxLengthTemp < PackageLength + 4)
    {
      break;
    }
    //Now, HeaderIndex point32_t to¡®0xAA¡¯
    //Head fram	  PackageLength	 DataNo	       Data	       ChkSum
    //0xAA,0x55	     HB,LB         2B   (ChNum*N*DNpCH) B	 1B
    //--------------------------------------------------------------------------------
    //Save data	in order
    uint64_t MoveIndex = HeaderIndex+6;
    if(MoveIndex >=  FORCE_BUFFER_SIZE)
    {
      MoveIndex = MoveIndex -  FORCE_BUFFER_SIZE;
    }
    unsigned char CheckSum = 0x00;
    for(i = 0x00; i <  PackageLength-3; i++)
    {
      CheckSum += g_ForceMyRxBuffer[MoveIndex];
      MoveIndex++;
      if(MoveIndex == FORCE_BUFFER_SIZE)
      {
        MoveIndex = 0x00;
      }
    }
    if(CheckSum != g_ForceMyRxBuffer[MoveIndex])
    {
      g_DataPtrOut = MoveIndex;//ÖžÏòchecksum
      break;
    }
    //---------------------------------------------------------------------------------
    unsigned char DataTemp[12] = {0};//ÁÙÊ±±£ŽæÊýŸÝ£¬6ÍšµÀ£¬Ò»žöµã
    MoveIndex = HeaderIndex+6;
    if(MoveIndex >= FORCE_BUFFER_SIZE)
    {
      MoveIndex = MoveIndex - FORCE_BUFFER_SIZE;
    }
    for(i = 0x00; i < PackageLength-3; i++)
    {
      DataTemp[i] = g_ForceMyRxBuffer[MoveIndex++];
      if(MoveIndex == FORCE_BUFFER_SIZE)
      {
        MoveIndex = 0x00;
      }
    }
    g_DataPtrOut = MoveIndex;
    //---------------------------------------------------------------------------------
    //ŒÆËãADcounts
    int32_t ADCounts[6] = {0};//
    for( i =0; i < 6; i++ )
    {
      ADCounts[i] = DataTemp[2*i]<<8 |  DataTemp[2*i+1];
    }
    //ŒÆËãÃ¿žöÍšµÀ
    double	ChValue[6]={0.0};
    for ( i = 0; i < 6; i++)
    {
      ChValue[i] = 1000.0 * ( ADCounts[i] - m_dAmpZero[i]) / (double)65535.0 * (double)5  / m_dChnGain[i] / m_dChnEx[i];
    }
    //ÀûÓÃœâñîŸØÕóŒÆËãÁŠÖµ
    double ForceTemp[6] = {0.0};
    for( i = 0x00; i < 6; i++)
    {
      ForceTemp[i] =   ChValue[0]*m_dDecouplingCoefficient[i][0] + ChValue[1]*m_dDecouplingCoefficient[i][1]
          + ChValue[2]*m_dDecouplingCoefficient[i][2] + ChValue[3]*m_dDecouplingCoefficient[i][3]
          + ChValue[4]*m_dDecouplingCoefficient[i][4] + ChValue[5]*m_dDecouplingCoefficient[i][5];
    }
    //±£ŽæÊýŸÝ£¬g_ForceIndexÊŒÖÕÖžÏò×îÐÂÒ»×éÊýŸÝ
    uint64_t ForceNextIndex = g_ForceIndex + 1;//ÖžÏòÏÂÒ»žöŽæŽ¢Î»ÖÃ£¬×¢Òâg_ForceIndex¶šÒå
    if( ForceNextIndex >= FORCE_BUFFER_SIZE)
      ForceNextIndex = ForceNextIndex - FORCE_BUFFER_SIZE;//×¢ÒâÑ­»·±£ŽæµÄË÷Òý

//    force_mutex_.lock();
    for( i = 0; i <6; i++)
    {
      g_ForceBuffer[i] = ForceTemp[i];//±£ŽæÊýŸ
    }
    debug_pub_.publish(empty_);
//    force_mutex_.unlock();

    static bool is_first = true;
    if(is_first) {
      bias_ = g_ForceBuffer[2];
      is_first = false;
    }
    g_ForceIndex = ForceNextIndex;//±£ŽæÓÐÐ§ºó£¬ÖžÏòÏÂÒ»žöÓÐÐ§ÊýŸÝ£¬ÆäËûÏß³ÌÍš¹ýg_ForceIndex·ÃÎÊÊýŸÝ

    //CString str;
    //str.Format( _T("g_ComputingTime=%.5f\n,g_ForceIndex=%d"),	g_ComputingTime, ForceNextIndex );
    //GetDlgItem(IDC_EDIT2)->SetWindowText(str);
  }
  return true;
}

double ForceSensor::getZForce()
{
//  force_mutex_.lock();
  double result = g_ForceBuffer[2] - bias_;
//  force_mutex_.unlock();
  return result;
}

}














