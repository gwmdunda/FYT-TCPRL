/*
 * softmax-lte.cc
 *
 *  Created on: Oct 27, 2020
 *      Author: tcp
 */

/*
 * test-app.cc
 *
 *  Created on: Oct 7, 2020
 *      Author: tcp
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/traffic-control-module.h"
#include <cmath>
#include <fstream>
#include "ns3/gnuplot.h"


using namespace ns3;

//------------------------------ Global variable definition starts here ------------------------------
uint8_t readThroughput; //If you enter a number larger than the current UE ID it will not display
bool readQueue;
uint8_t readReward; //If you enter a number larger than the current UE ID it will not display
bool readCWND;
bool readPolicy;
bool readBias;
uint8_t observationId;
uint8_t whicheNB;
bool verboseTr;

uint8_t idCounter = 0;

std::vector<std::pair<Time, uint32_t>> trackCWND;
std::vector<std::pair<Time, uint32_t>> trackCWND2;
std::vector<std::pair<Time, uint32_t>> trackCWND3;
std::vector<uint32_t> trackQueueSize;
double trackExpectedThroughput;
std::vector<std::pair<Time, double>> trackCalculatedThroughput;
std::vector<uint32_t> queue1;
std::vector<uint32_t> queue2;
std::vector<uint32_t> queue3;
std::vector<std::pair<double, double>> trackState;

struct Timeout{
	Time time;
};
//------------------------------ Global variable definition ends here --------------------------------

//------------------------------ Global function definition ends here --------------------------------
void changeDataRate(){
	Config::Set("/NodeList/1/DeviceList/1/$ns3::PointToPointNetDevice/ReceiveErrorModel/$ns3::RateErrorModel/ErrorRate", DoubleValue(0.012));
}
//------------------------------ Global function definition ends here --------------------------------

//------------------------------ TCP Control Enum definition starts here --------------------------
enum TCPControl {TCPRL, TCPReno, TCPCubic};
//------------------------------ TCP Control Enum definition ends here --------------------------
//-----------------------------TimestampTag class definition starts here -------------------------
class TimestampTag : public Tag {
public:
	static TypeId GetTypeId(void);
	virtual TypeId GetInstanceTypeId (void) const;

	virtual uint32_t GetSerializedSize (void) const;
	virtual void Serialize (TagBuffer i) const;
	virtual void Deserialize (TagBuffer i);

	void SetTimestamp (Time time);
	Time GetTimestamp (void) const;
	void Print (std::ostream &os) const;
private:
	Time m_timestamp;
};
TypeId
 TimestampTag::GetTypeId (void)
 {
   static TypeId tid = TypeId ("TimestampTag")
     .SetParent<Tag> ()
     .AddConstructor<TimestampTag> ()
     .AddAttribute ("Timestamp",
                    "Some momentous point in time!",
                    EmptyAttributeValue (),
                    MakeTimeAccessor (&TimestampTag::GetTimestamp),
                    MakeTimeChecker ())
   ;
   return tid;
 }
 TypeId
 TimestampTag::GetInstanceTypeId (void) const
 {
   return GetTypeId ();
 }

 uint32_t
 TimestampTag::GetSerializedSize (void) const
 {
   return 8;
 }
 void
 TimestampTag::Serialize (TagBuffer i) const
 {
   int64_t t = m_timestamp.GetNanoSeconds ();
   i.Write ((const uint8_t *)&t, 8);
 }
 void
 TimestampTag::Deserialize (TagBuffer i)
 {
   int64_t t;
   i.Read ((uint8_t *)&t, 8);
   m_timestamp = NanoSeconds (t);
 }

 void
 TimestampTag::SetTimestamp (Time time)
 {
   m_timestamp = time;
 }
 Time
 TimestampTag::GetTimestamp (void) const
 {
   return m_timestamp;
 }

 void
  TimestampTag::Print (std::ostream &os) const
  {
    os << "t=" << m_timestamp;
  }
 //-----------------------------TimestampTag class definition ends here -------------------------

//-----------------------------CustomECNTag class definition starts here -------------------------

class CustomECNTag : public Tag {
private:
	uint8_t ECNbits; //0x01 corresponds to CE bit while 0x10 corresponds to ECT bit
										//When in congestion mode 0x11.

public:
	static TypeId GetTypeId(){
		static TypeId tid = TypeId("ns3::CustomECNTag").SetParent<Tag>().AddConstructor<CustomECNTag>();
		return tid;
	}
	virtual TypeId GetInstanceTypeId() const { return GetTypeId(); }
	virtual uint32_t GetSerializedSize() const { return 2*sizeof(bool);}
	virtual void Serialize(TagBuffer buffer) const { buffer.WriteU8(ECNbits); }
	virtual void Deserialize(TagBuffer buffer) { ECNbits =  buffer.ReadU8(); }
	virtual void Print(std::ostream &os) const { os << "ECN Bits : " << ECNbits; }
	void setBits (uint8_t token) { ECNbits = token; }
	uint8_t getBits() {return ECNbits;}
	bool isCongested() { return ECNbits == 3 ; }
	bool isECNCapable() {return ECNbits | 0x11; }
	bool isPositiveReward() {
		if (ECNbits == 2) return true;
		else return false;
	}


};
//-----------------------------CustomECNTag class definition ends here -------------------------

//-----------------------------CustomTCPTag class definition starts here -------------------------

class CustomTCPTag : public Tag {
private:
	uint8_t nodeId;
public:
	static TypeId GetTypeId(){
		static TypeId tid = TypeId("ns3::CustomTCPTag").SetParent<Tag>().AddConstructor<CustomTCPTag>();
		return tid;
	}
	virtual TypeId GetInstanceTypeId() const { return GetTypeId(); }
	virtual uint32_t GetSerializedSize() const { return sizeof(uint8_t);}
	virtual void Serialize(TagBuffer buffer) const { buffer.WriteU8(nodeId); }
	virtual void Deserialize(TagBuffer buffer) { nodeId =  buffer.ReadU8(); }
	virtual void Print(std::ostream &os) const { os << "Node Id : " << nodeId; }
	void setId (uint8_t token) { nodeId = token; }
	uint8_t getId() {return nodeId;}
};

//-----------------------------CustomTCPTag class definition ends here -------------------------

//------------------------------EnbApp class definition starts here ------------------------------------

class EnbApp : public Application
{
public:
	EnbApp();
	virtual ~EnbApp();

	void Setup (std::vector <std::pair<Address, uint8_t>> addressNodePairs,
			 uint16_t ulPortOffset, uint16_t dlPortSGW, uint32_t packetSize,
			float m_eta, Time timeDelay, uint16_t maxQueue, float dropP, Address server, uint16_t m_serverPort, bool isExp4, Time period);

protected:
	void handleRead(Ptr<Socket> socket);
	void handleReadServer(Ptr<Socket> socket);
	void handleAccept(Ptr<Socket> socket, const Address& from);
	//TODO: Split handleAccept from UE and SGW
	/*void handlePeerClose(Ptr<Socket> socket);
	void handlePeerError(Ptr<Socket> socket);*/

private:
	virtual void StartApplication (void);
	virtual void StopApplication (void);
	void queueBecomeCongested(Ptr<const Packet> packet);
	void ScheduleTx (void);
	void SendPacketToUE (uint8_t nodeId, CustomECNTag ecnTag, TimestampTag timeTag);
	void SendPacketToServer (uint8_t nodeId, CustomECNTag ecnTag, TimestampTag timeTag);
	void setExpectedThroughput(bool isComingUE);
	void exp1ChangeQueueSize();
	void exp1ChangeQueueTime();
	void updateUEStatus();


	std::vector<Ptr<Socket>>     m_socketReceivers;
	std::vector<Ptr<Socket>>  m_socketSenders;
    std::vector<Time> m_lastReceivedTimes;
	std::vector <std::pair<Address, uint8_t>> m_addressNodePairs;
	uint32_t        m_packetSize;
	DataRate        m_dataRate;
	bool            m_running;
	uint16_t m_ulPortOffset;
	uint16_t m_dlPortSGW;
	float  m_eta;
	Time m_serviceTime;
	uint32_t m_queueCounter;
	uint16_t m_maxQueueSize;
    float m_dropProbability;
    Ptr<UniformRandomVariable> sample;
    uint8_t m_numberOfUE;
    uint32_t m_expectedQueueUtilization; //Should be the same across the users to ensure fairness
    std::vector<uint32_t> m_queueUtil;
    Address m_server;
    Ptr<Socket> m_socketToServer;
    Ptr<Socket> m_socketFromServer;
    uint16_t m_serverPort; //Uplink server port
    bool m_isExp4;
    Time m_period;
    bool m_exp4State;
    uint8_t m_eNBID;
    uint8_t m_prevMax;
};

EnbApp::EnbApp ()
{
}

EnbApp::~EnbApp()
{
  m_socketSenders.clear();
  m_socketReceivers.clear();
}

void
EnbApp::Setup (std::vector <std::pair<Address, uint8_t>> addressNodePairs,
		uint16_t ulPortOffset, uint16_t dlPortSGW, uint32_t packetSize,
		float eta, Time timeDelay, uint16_t maxQueue, float dropP, Address server, uint16_t serverPort, bool isExp4, Time period)
{
  m_addressNodePairs = addressNodePairs;
  m_ulPortOffset = ulPortOffset;
  m_dlPortSGW = dlPortSGW;
  m_eta = eta;
  m_packetSize = packetSize;
  m_serviceTime = timeDelay;
  m_maxQueueSize = maxQueue;
  if(dropP > 1 || dropP < 0){
	  std::cerr<<"Illegal value for dropping probability"<<std::endl;
	  exit(-1);
  }
  m_dropProbability = dropP;
  m_server = server;
  m_serverPort = serverPort;
  m_isExp4 = isExp4;
  m_period = period;
  m_exp4State = true;
  m_eNBID = idCounter;
  idCounter++;
  m_prevMax = 99;
}

void
EnbApp::setExpectedThroughput(bool isComingUE){
	if(isComingUE){
		m_numberOfUE++;
		if(m_numberOfUE == 1) m_expectedQueueUtilization = m_maxQueueSize/2;
		else m_expectedQueueUtilization =(m_numberOfUE - 1)* m_expectedQueueUtilization /m_numberOfUE;
		m_queueUtil.push_back(0);
	}
	else{
		m_numberOfUE--;
		//TODO: handle deletion by id of USER, only worry this part when we are in handover stage
		if(m_numberOfUE != 0){
			m_expectedQueueUtilization = (m_numberOfUE + 1)*m_expectedQueueUtilization/m_numberOfUE;
		}
	}
	trackExpectedThroughput = m_expectedQueueUtilization;
	if(m_isExp4){
			Simulator::Schedule(m_period, &EnbApp::setExpectedThroughput, this, !m_exp4State);
			m_exp4State = !m_exp4State;
		}
}
void
EnbApp::StartApplication (void)
{
  std::cout<< " Starting ENB Application" << std::endl;
  m_running = true;
  m_numberOfUE = 0;
  m_queueCounter = 0;
  Time current_time = Simulator::Now();
  m_lastReceivedTimes.resize(m_addressNodePairs.size(), current_time);

  for(auto it = m_addressNodePairs.begin(); it != m_addressNodePairs.end(); ++it){
	  Ptr<Socket> tcpSenderSocket = Socket::CreateSocket (this->GetNode(), UdpSocketFactory::GetTypeId ());
	  tcpSenderSocket->Bind();
	  tcpSenderSocket->Connect (it->first);
	  tcpSenderSocket->ShutdownRecv();
	  m_socketSenders.push_back(tcpSenderSocket);

	  Ptr<Socket> tcpReceiverSocket = Socket::CreateSocket(this->GetNode(), UdpSocketFactory::GetTypeId());
	  tcpReceiverSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_ulPortOffset + it->second));
	  tcpReceiverSocket->ShutdownSend();
	  tcpReceiverSocket->SetRecvCallback(MakeCallback(&EnbApp::handleRead, this));
	  m_socketReceivers.push_back(tcpReceiverSocket);

	  setExpectedThroughput(true);
  }
    m_socketToServer = Socket::CreateSocket(this->GetNode(), UdpSocketFactory::GetTypeId());
    m_socketToServer->Bind();
    m_socketToServer->Connect(m_server);
    m_socketToServer->ShutdownRecv();

    m_socketFromServer = Socket::CreateSocket(this->GetNode(), UdpSocketFactory::GetTypeId());
    m_socketFromServer->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_serverPort));
    m_socketFromServer->ShutdownSend();
    m_socketFromServer->SetRecvCallback(MakeCallback(&EnbApp::handleReadServer, this));

    sample = CreateObject<UniformRandomVariable>();
	sample->SetAttribute("Min", DoubleValue(0));
	sample->SetAttribute("Max", DoubleValue(1));

	Simulator::Schedule (m_serviceTime, &EnbApp::updateUEStatus, this);
	//Simulator::Schedule (Seconds(50), &EnbApp::exp1ChangeQueueSize, this);
	//Simulator::Schedule (Seconds(100), &EnbApp::exp1ChangeQueueTime, this);

}
void
EnbApp::handleReadServer(Ptr<Socket> socket){
	++m_queueCounter;
	trackQueueSize.push_back(m_queueCounter);
	if(!m_running) {return;}

	Ptr<Packet> packet;
	Address sender;
	while((packet = socket->RecvFrom(sender))){
		CustomTCPTag  tcpTag;
		TimestampTag timeTag;
		CustomECNTag ecnTag;
		packet->PeekPacketTag(ecnTag);
		packet->PeekPacketTag(tcpTag);
		packet->PeekPacketTag(timeTag);
		uint8_t ueId = tcpTag.getId();
		if(!ecnTag.isCongested()&&m_queueCounter > m_maxQueueSize){
					 double val = sample->GetValue();
					 if(val < m_dropProbability) {
						 --m_queueCounter;
						 return;}
		}
		Simulator::Schedule (m_queueCounter*m_serviceTime, &EnbApp::SendPacketToUE, this, ueId, ecnTag, timeTag);
	}

}

void
EnbApp::handleRead(Ptr<Socket> socket){
	++m_queueCounter;
	trackQueueSize.push_back(m_queueCounter);
	if(!m_running) {return;}
	Ptr<Packet> packet;
	Address sender;
	while((packet = socket->RecvFrom(sender))){
		CustomECNTag ecnTag;
		CustomTCPTag  tcpTag;
		TimestampTag timeTag;
		packet->PeekPacketTag(ecnTag);
		packet->PeekPacketTag(tcpTag);
		packet->PeekPacketTag(timeTag);

		uint8_t ueId = tcpTag.getId();
		m_queueUtil[ueId] += 1;

		if(m_queueCounter > m_maxQueueSize){
			 double val = sample->GetValue();
			 uint16_t maxIndex = max_element(m_queueUtil.begin(), m_queueUtil.end()) - m_queueUtil.begin();
			 ecnTag.setBits(3); //Congested State
			 if(maxIndex == ueId /*&& maxIndex != m_prevMax*/){
				 std::cout << "Max index = " << (int)maxIndex << std::endl;
				 std::cout << "queue 0  = " << (int)m_queueUtil[0] << std::endl;
				 std::cout << "queue 1 = " << (int)m_queueUtil[1] << std::endl;
				 std::cout << "queue 2 = " << (int)m_queueUtil[2] << std::endl;
				 m_queueUtil[ueId] -= 1;
				 Simulator::Schedule (m_queueCounter*m_serviceTime, &EnbApp::SendPacketToUE, this, ueId, ecnTag, timeTag);
			    m_prevMax = maxIndex;
				return;
			 }
			 if(val < m_dropProbability) {
					 --m_queueCounter;
					 m_queueUtil[ueId] -= 1;
					 return;
			 	}
			Simulator::Schedule (m_queueCounter*m_serviceTime, &EnbApp::SendPacketToServer, this, ueId, ecnTag, timeTag);
			return;
		}
		if(ecnTag.isPositiveReward()){
			if(m_queueUtil[ueId] > m_expectedQueueUtilization|| m_queueUtil[ueId] < (m_eta*m_expectedQueueUtilization)){
				//Send to UE directly
				ecnTag.setBits(1); //You are penalized!
			}
		}
		Simulator::Schedule (m_queueCounter*m_serviceTime, &EnbApp::SendPacketToServer, this, ueId, ecnTag, timeTag);

	}

}

void
EnbApp::updateUEStatus(){
		Time current_time = Simulator::Now();
		if(readThroughput < m_numberOfUE && m_eNBID == whicheNB){
			if(verboseTr){
				std::cout<<"Current queue usage for user  "<<(int)readThroughput<<" is : "<<m_queueUtil[readThroughput] << std::endl;
				std::cout<<"Expected queue usage per user:"<<m_expectedQueueUtilization<<std::endl;
			}
				trackCalculatedThroughput.push_back(std::make_pair(current_time, m_queueUtil[readThroughput]));
		}
		queue1.push_back(m_queueUtil[0]);
		queue2.push_back(m_queueUtil[1]);
		queue3.push_back(m_queueUtil[2]);
		Simulator::Schedule (m_serviceTime*10, &EnbApp::updateUEStatus, this);
}
void
EnbApp::SendPacketToUE(uint8_t nodeId, CustomECNTag ecnTag, TimestampTag timeTag){
	if(!m_running) {return;}
	--m_queueCounter;
	Ptr<Packet> packet = Create<Packet> (m_packetSize);
	packet->AddPacketTag(ecnTag);
	packet->AddPacketTag(timeTag);

	CustomTCPTag tcpTag;
	tcpTag.setId(nodeId);
	packet->AddPacketTag(tcpTag);
	Ptr<Socket> sendSocket = m_socketSenders[nodeId];

	sendSocket->Send(packet);

}

void
EnbApp::SendPacketToServer (uint8_t nodeId, CustomECNTag ecnTag, TimestampTag timeTag){
	if(!m_running) {return;}
	--m_queueCounter;
	m_queueUtil[nodeId] -= 1;

	Ptr<Packet> packet = Create<Packet> (m_packetSize);
	packet->AddPacketTag(ecnTag);
	packet->AddPacketTag(timeTag);
	CustomTCPTag tcpTag;
	tcpTag.setId(nodeId);
	packet->AddPacketTag(tcpTag);
	m_socketToServer->Send(packet);

}
void
EnbApp::StopApplication (void)
{
  m_running = false;
  for(auto it =m_socketSenders.begin(); it != m_socketSenders.end() ; ++it){
	  (*it)->Close();
  }
  for(auto it =m_socketReceivers.begin(); it != m_socketReceivers.end() ; ++it){
  	  (*it)->Close();
    }

}
//------------------------------EnbApp class definition ends here ------------------------------------

//------------------------------State struct definition starts here ------------------------------------
struct State{
		double ACKRatio;
		double RTTRatio;
		double cwndRatio;
		bool isTerminal;
	};
//------------------------------State struct definition ends here ------------------------------------


//------------------------------- NN class definition starts here --------------------------------------

class NN{ //Assuming one hidden layer only and ReLu non-linearity
public:
	NN(double alpha_w, double hidden_size);
	double getValue(std::vector<double> v);
	void updateWeights(double tdError, std::vector<double> v);

private:
	double dotProduct(std::vector<double> v1, std::vector<double> v2);
	void initializeWeights();
    double sigmoid(double x);

	double m_alphaW;
	double m_hiddenSize;
	std::vector<std::vector<double>>m_weight0;
	std::vector<double> m_bias0;
	std::vector<double>m_weight1;
	double m_bias1;

};

NN::NN(double alpha_w, double hidden_size){
	m_alphaW = alpha_w;
	m_hiddenSize = hidden_size;
	initializeWeights();
}

void
NN::initializeWeights(){
	//Assuming the feature vector used has three elements: first two elements in state and change in cwnd
	Ptr<NormalRandomVariable> sample = CreateObject<NormalRandomVariable>();
	sample->SetAttribute("Mean", DoubleValue(0));
	sample->SetAttribute("Variance", DoubleValue(2.0/(4.0*static_cast<double>(m_hiddenSize))));
	m_bias0.resize(m_hiddenSize, 0);

	m_weight0.resize(m_hiddenSize);
	m_weight1.resize(m_hiddenSize, 0);
	for(uint32_t i =0; i < m_weight0.size(); ++i){
		m_weight0[i].resize(3,0);
		m_bias0[i] = sample->GetValue();
		for(uint8_t j; j < 3; ++j){
			m_weight0[i][j] = sample->GetValue();
		}
	}
	sample->SetAttribute("Variance", DoubleValue(1/(1.0 + static_cast<double>(m_hiddenSize))));
	m_bias1 = 0;
	for(uint32_t i =0; i < m_weight0.size(); ++i){
		m_weight1[i] = sample->GetValue();
	}

}
double
NN::dotProduct(std::vector<double> v1, std::vector<double> v2){
	if(v1.size() != v2.size()){
		std::cout<<"Vector sizes do not match!"<<std::endl;
		exit(-1);
	}
	double sum = 0;
	for(uint32_t i = 0; i < v1.size(); ++i){
		sum += v1[i]*v2[i];
	}
	return sum;
}

double
NN::sigmoid(double x){
	return 1/(1+std::exp(-x));
}

double
NN::getValue(std::vector<double> v){
	if(v.size() != 3){
		std::cout<<"Unsupported number of inputs! Your input has "<<v.size()<<" elements"<<std::endl;
		exit(-1);
	}
	//Calculate the output from hidden layer
	std::vector<double> hiddenOut;
	for(int i = 0; i < m_hiddenSize; ++i){
		double intermediate = dotProduct(v, m_weight0[i]) + m_bias0[i];
		//double res = intermediate > 0 ? intermediate : 0; //we will use ReLu as non-linear output
		double res = sigmoid(intermediate); //We will use sigmoid as non-linear output
		hiddenOut.push_back(res);
	}
	//calculate the output
	return (dotProduct(hiddenOut, m_weight1) + m_bias1 );
}

void
NN::updateWeights(double tdError, std::vector<double> v){
	if(readBias){
		std::cout<<"Bias layer 0: ";
		for(uint8_t i = 0; i < m_bias0.size(); ++i){
			std::cout<<m_bias0[i]<<"   ";
		}
		std::cout<<std::endl;
		std::cout<<"Bias layer 1: "<< m_bias1 << std::endl;
	}
	m_bias1 += m_alphaW*tdError;
	/*for(int i = 0; i < m_hiddenSize; ++i){
			double intermediate = dotProduct(v, m_weight0[i]) + m_bias0[i];
			int res = intermediate > 0 ? 1 : 0; //we will use ReLu as non-linear output
			double r = intermediate > 0 ? intermediate : 0;
			m_bias0[i] += m_alphaW*res*m_weight1[i]*tdError;
			for(uint8_t j = 0; j < v.size(); ++j){
				m_weight0[i][j] += m_alphaW*v[j]*res*m_weight1[i]*tdError;
			}
			m_weight1[i] += m_alphaW*r*tdError;
	}*/
	for(int i = 0; i < m_hiddenSize; ++i){
		double intermediate = dotProduct(v, m_weight0[i]) + m_bias0[i];
		double res = sigmoid(intermediate);
		m_weight1[i] += m_alphaW*res*tdError;
		m_bias0[i] += m_alphaW*res*(1.0-res)*m_weight1[i]*tdError;
		for(uint8_t j = 0; j < v.size(); ++j){
						m_weight0[i][j] += m_alphaW*v[j]*res*(1.0-res)*m_weight1[i]*tdError;
				}
	}

}
//------------------------------- NN class definition ends here ----------------------------------------

//------------------------------RLCompute class definition starts here -----------------------------

class RLCompute {
public:
	RLCompute(//int action_bound,
			double alphaW, uint16_t hiddenSize,  std::vector<double> alphaTheta, double gamma, uint8_t userid);
	int compute(float reward, State currState, uint32_t cwndMax);
	void resetPolicyParams();

private:
	int actionSelect(State state);
	void updateTheta();
	double getValueFunction(State state);
	void updateStochasticPolicy(State state);
	std::vector<double> getFeatureVector(State s);

	std::vector<double> m_thetaRTT;
	std::vector<double> m_thetaACK;
	std::vector<double> m_thetaCWND;
	std::vector<double> m_policy;  //softmax: [-sqrt(cwnd), -1, 0 , +1, +sqrt(cwnd)]
	const uint8_t m_numOfAction = 5;
	std::vector<double> m_alphaTheta;
	float m_gamma;
	State m_lastState;
	int m_lastAction;
	bool m_firstTime;
	bool m_isTerminal;
	NN* m_nn;
	uint8_t m_userid;
	uint32_t m_cwndMax;

};
RLCompute::RLCompute(//int action_bound,
		double alphaW, uint16_t hiddenSize, std::vector<double> alphaTheta, double gamma, uint8_t user_id){
//	m_actionBound = action_bound;
	m_thetaACK.reserve(m_numOfAction);
	m_thetaRTT.reserve(m_numOfAction);
	m_thetaCWND.reserve(m_numOfAction);
	m_alphaTheta = alphaTheta;
	resetPolicyParams();
    m_nn = new NN(alphaW, hiddenSize);
	m_gamma = gamma;
	m_firstTime = true;
	m_isTerminal = false;
	m_lastAction = 0;
	m_userid = user_id;

}

void
RLCompute::resetPolicyParams(){
	 Ptr<UniformRandomVariable> sampleX = CreateObject<UniformRandomVariable>();

		sampleX->SetAttribute("Min", DoubleValue(-0.2));
		sampleX->SetAttribute("Max", DoubleValue(-0.1));
		m_thetaACK[0] = sampleX->GetValue();    // change = -sqrt(cwnd)
		m_thetaRTT[0] = sampleX->GetValue();
		m_thetaCWND[0] = sampleX->GetValue();

		sampleX->SetAttribute("Min", DoubleValue(-0.1));
		sampleX->SetAttribute("Max", DoubleValue(0));
		m_thetaACK[1] = sampleX->GetValue();    // change = -1
		m_thetaRTT[1] = sampleX->GetValue();
		m_thetaCWND[1] = sampleX->GetValue();

		sampleX->SetAttribute("Min", DoubleValue(-0.1));
		sampleX->SetAttribute("Max", DoubleValue(0.1));
		m_thetaACK[2] = sampleX->GetValue();     // no change
		m_thetaRTT[2] = sampleX->GetValue();
		m_thetaCWND[2] = sampleX->GetValue();

		sampleX->SetAttribute("Min", DoubleValue(0));
		sampleX->SetAttribute("Max", DoubleValue(0.1));
		m_thetaACK[3] = sampleX->GetValue();        //change = +1
		m_thetaRTT[3] = sampleX->GetValue();
		m_thetaCWND[3] = sampleX->GetValue();

		sampleX->SetAttribute("Min", DoubleValue(0));
		sampleX->SetAttribute("Max", DoubleValue(0.1));
		m_thetaACK[4] = sampleX->GetValue();         //change = +sqrt(cwnd)
		m_thetaRTT[4] = sampleX->GetValue();
		m_thetaCWND[4] = sampleX->GetValue();
}

int
RLCompute::compute(float reward, State currState, uint32_t cwndMax){
	m_cwndMax = cwndMax;
	if(m_isTerminal){
		if(currState.isTerminal){
			return m_lastAction;
		}
		m_isTerminal = false;
		m_firstTime = true;
	}
	if(m_firstTime){
		m_lastState = currState;
		updateStochasticPolicy(currState);
		m_lastAction = actionSelect(m_lastState);
		m_firstTime = false;
		return m_lastAction;
	}
	if(currState.isTerminal){
		std::cout<<"Agent stops learning"<<std::endl;
		double td_error = reward  - getValueFunction(m_lastState);//getValueFunction(m_lastState, m_lastAction);
		//m_nn->updateWeights(td_error, getFeatureVector(m_lastState, m_lastAction));
		m_nn->updateWeights(td_error, getFeatureVector(m_lastState));
		m_isTerminal = true;
		return 0;
	}
	updateTheta();
	updateStochasticPolicy(currState);
	int next_action =  actionSelect(m_lastState);
	double td_error = reward + m_gamma*getValueFunction(currState) - getValueFunction(m_lastState);//getValueFunction(m_lastState, m_lastAction);
	//m_nn->updateWeights(td_error, getFeatureVector(m_lastState, m_lastAction));
	m_nn->updateWeights(td_error, getFeatureVector(m_lastState));
	if(readPolicy && m_userid == observationId){
		std::cout << "Theta ACK" << std::endl;
		std::cout<<"[ ";
		for(int i = 0; i < m_numOfAction-1 ;++i){
			std::cout<<m_thetaACK[i]<<" , ";
		}
		std::cout<<m_thetaACK[m_numOfAction-1]<<" ]"<<std::endl;
		std::cout << "Theta RTT" << std::endl;
		std::cout<<"[ ";
		for(int i = 0; i < m_numOfAction-1 ;++i){
			std::cout<<m_thetaRTT[i]<<" , ";
		}
		std::cout<<m_thetaRTT[m_numOfAction-1]<<" ]"<<std::endl;
		std::cout << "Theta CWND" << std::endl;
				std::cout<<"[ ";
				for(int i = 0; i < m_numOfAction-1 ;++i){
					std::cout<<m_thetaCWND[i]<<" , ";
				}
				std::cout<<m_thetaCWND[m_numOfAction-1]<<" ]"<<std::endl;
		std::cout << "PMF" << std::endl;
		std::cout<<"[ ";
		for(int i = 0; i < m_numOfAction-1 ;++i){
					std::cout<<m_policy[i]<<" , ";
				}
				std::cout<<m_policy[m_numOfAction-1]<<" ]"<<std::endl;
		std::cout<<"Value function = "<< getValueFunction(currState) << std::endl;
	}
	m_lastState = currState;
	m_lastAction = next_action;
	return next_action;
}

void
RLCompute::updateTheta(){
	double Q = getValueFunction(m_lastState);
	for(int i =0; i <m_numOfAction; ++i){
		m_thetaACK[i] += m_alphaTheta[0]*Q*m_lastState.ACKRatio*(1-m_policy[i]) ;
		m_thetaRTT[i] += m_alphaTheta[1]*Q*m_lastState.RTTRatio*(1-m_policy[i]);
		m_thetaCWND[i] += m_alphaTheta[2]*Q*m_lastState.cwndRatio*(1-m_policy[i]);
	}
}


void
RLCompute::updateStochasticPolicy(State state){
	 std::vector<double> h;
	 double maxVal = -10000000;
	 for(int i = 0; i < m_numOfAction; ++i){
		 double temp = m_thetaACK[i]*state.ACKRatio + m_thetaRTT[i]*state.RTTRatio+m_thetaCWND[i]*state.cwndRatio;
		 if(temp > maxVal) maxVal = temp;
		 h.push_back(temp);
	 }
	 double normalization = 0;
	 for(int i = 0; i <m_numOfAction; ++i){
		 h[i] -= maxVal;
		 h[i] = std::exp(h[i]);
		 normalization += h[i];
	 }
	 for(int i = 0; i <m_numOfAction; ++i){
		 h[i] = h[i]/normalization;
	 }
	m_policy = h;

}

std::vector<double>
RLCompute::getFeatureVector(State s){
	std::vector<double> inp;
	inp.push_back(s.ACKRatio);
	inp.push_back(s.RTTRatio);
	inp.push_back(s.cwndRatio);
	return inp;
}

double
RLCompute::getValueFunction(State state){
	return m_nn->getValue(getFeatureVector(state));
}
int
RLCompute::actionSelect(State state){
	Ptr<UniformRandomVariable> sample = CreateObject<UniformRandomVariable>();
		sample->SetAttribute("Min", DoubleValue(0));
		sample->SetAttribute("Max", DoubleValue(1));
		double rand_number = sample->GetValue();
		double sum = 0;
		uint8_t index = 0;
		for(int i = 0; i < m_numOfAction; ++i){
			sum += m_policy[i];
			if(sum > rand_number) {
				index = i;
				break;
			}
		}
		switch(index){
			case 0:
				//return -floor(sqrt(state.cwndRatio*m_cwndMax));
				return -1;
			case 1:
				return 0;
			case 2:
				return 1;
			case 3:
				return 2;
			case 4:
				//return floor(sqrt(state.cwndRatio*m_cwndMax));
				return 3;
		}
		return 0;

}

//------------------------------RLCompute class definition ends here -----------------------------

//------------------------------RenoCompute class definition starts here ------------------------
class RenoCompute{ //Note we do not consider fast recovery since we always assume no dups and always arrive in order
private:
	uint32_t m_ssThresh;
	uint32_t m_initialCwnd;

public:
	void setSSThresh(uint32_t ssThresh);
	uint32_t update(uint32_t cwnd, uint32_t countPacket); //note that this return cwnd instead of its change
	RenoCompute();
	uint32_t retransmit(uint32_t cwnd);

};

RenoCompute::RenoCompute(){
	 m_ssThresh = 1e6;
	 m_initialCwnd = 1;

}

void
RenoCompute::setSSThresh(uint32_t ssThresh){
	m_ssThresh = ssThresh;
}

uint32_t
RenoCompute::update(uint32_t cwnd, uint32_t countPacket){
	uint32_t new_cwnd;
	if(cwnd < m_ssThresh){ //slow start mode
			new_cwnd = cwnd *2;
	}
	else{ //congestion avoidance mode
		double adder = 1;
		new_cwnd = cwnd + static_cast<uint32_t> (adder);
	}
	return new_cwnd;
}

uint32_t
RenoCompute::retransmit(uint32_t cwnd){ //Called when there is a congestion notification
	m_ssThresh =  cwnd*0.7;
	return cwnd*0.7;
}

//------------------------------RenoCompute class definition ends here ------------------------
//------------------------------ TCP Cubic class definition starts here ----------------------------
class CubicCompute{
public:
	void Setup();
	uint32_t OnData(double minRTT, uint32_t cwnd, double RTT); //Called every decision time
	uint32_t OnPacketLoss(uint32_t cwnd);
	uint32_t CubicUpdate(uint32_t cwnd, double RTT);
	void CubicReset();

private:
	bool m_friendliness;
	float beta; //Multiplicative decrease factor after packet loss
	bool m_fastConvergence;
	float C; //Scaling factor

	uint32_t m_ssThresh;
	float K;
	float WlastMax;
	Time m_epochStart;
	uint32_t m_originPoint;
	float Wtcp;
	float m_minRTT;

};

void
CubicCompute::Setup(){
	//Use default values
	m_friendliness = true;
	beta = 0.2;
	m_fastConvergence = true;
	C = 0.4;
	CubicReset();
}

uint32_t
CubicCompute::OnData(double minRTT, uint32_t cwnd, double RTT){
	m_minRTT = minRTT;
	if(cwnd <= m_ssThresh) return cwnd+1;
	else return CubicUpdate(cwnd, RTT);
}

void
CubicCompute::CubicReset(){//Need to reset minRTT from UE side when using this function more than once
	WlastMax = 0;
	m_epochStart = Seconds(0);
	m_originPoint = 0;
	m_minRTT = 0;
	Wtcp = 0;
	K = 0;
	m_ssThresh = 1e6; //arbitrarily large number
}
uint32_t
CubicCompute::CubicUpdate(uint32_t cwnd, double RTT){
	if (m_epochStart <= Seconds(0)){
		m_epochStart = Simulator::Now();
		if(cwnd < WlastMax){
			K = std::pow((WlastMax - cwnd)/C, 1/3.0);
			m_originPoint = WlastMax;
		}
		else{
			K = 0;
			m_originPoint = cwnd;
		}
		Wtcp = cwnd;
	}
	float t = Simulator::Now().GetMilliSeconds() + m_minRTT - m_epochStart.GetMilliSeconds();
	uint32_t target = m_originPoint +std::pow( C*(t/1000.0-K),3);
	uint32_t cwnd_update;
	if(target > cwnd) cwnd_update = cwnd + (target - cwnd)/cwnd;
	else cwnd_update = cwnd + 0.01/cwnd;
	if(m_friendliness){
		Wtcp += 3*beta/(2-beta)*t/RTT;
		if(Wtcp > cwnd && Wtcp > target){
			cwnd_update = cwnd + (uint32_t) (Wtcp - cwnd)/cwnd;
		}
	}
	return cwnd_update;
}

uint32_t
CubicCompute::OnPacketLoss(uint32_t cwnd){
	m_epochStart = Seconds(0);
	if(cwnd < WlastMax && m_fastConvergence)
		WlastMax = (uint32_t) cwnd*(1 - beta)/2;
	else
		WlastMax = cwnd;
	m_ssThresh = (uint32_t) cwnd*(1-beta);
	return (uint32_t) cwnd*(1-beta);
}


//-----------------------------CubicCompute class definition ends here ----------------------------------

//------------------------------UEApp class definition starts here ------------------------------------

class UEApp : public Application{
public:
	UEApp();
	virtual ~UEApp();

	void Setup (Address send, Address receive,  uint32_t packetSize, uint8_t ueId, DataRate dataRate,  uint16_t ulPort, uint16_t dlPort, Time decisionPeriod,
			float positiveReward, float negativeReward, double alphaW, uint16_t hiddenSize, std::vector<double> alphaTheta, double gamma, TCPControl tcpCtrl,  bool isPeriodic, uint16_t period, uint16_t uniqueID, Timeout rto);
	double getRTT();
protected:
	void handleRead(Ptr<Socket> socket);
	void handleAccept(Ptr<Socket> socket, const Address& from);


private:
	virtual void StartApplication (void);
	virtual void StopApplication (void);

	void ScheduleTx (void);
	void ScheduleTxNoACK (void);
	void SendPacket ();
	void SendPacketTimeout ();
	void updateReward(void);

	Ptr<Socket>     m_socketReceiver;
	Ptr<Socket>  m_socketSender;
	Address  m_serverAddressSend;
	Address m_serverAddressReceive;
	uint8_t    m_ueId;
	uint32_t        m_packetSize;
	DataRate        m_dataRate;
	EventId         m_sendEvent;
	bool            m_running;
	uint16_t m_ulPort;
	uint16_t m_dlPort;
	float   m_averageReward;
	uint32_t m_countRx;
	uint32_t m_countTx;
	double  m_estimatedRTT; //in milliseconds
	double m_devRTT;
	double m_minRTT;
	uint32_t m_cwnd;
	uint32_t m_cwndMax;
	uint32_t m_cwndCounter;
	uint32_t m_RxcwndCounter;
	bool m_hasCalledNOAck;
	Time m_decisionPeriod;
	float m_positiveReward;
	float m_negativeReward;
	RLCompute* rl;
	RenoCompute* reno;
	CubicCompute* cubic;
	double m_alphaW;
	std::vector<double> m_alphaTheta;
	double m_gamma;
	uint16_t m_hiddenSize;
	bool m_isCongested;
	int m_actionBound;
	TCPControl m_tcpCtrl;
	bool m_ssMode;
	uint32_t m_ssThreshRL;

	bool m_isOn;
	bool m_isPeriodic;
	uint16_t m_clockCounter;
	uint16_t m_period;
	uint16_t m_uniqueID;
	EventId m_timeoutEventId;
	Timeout m_to;
	Time m_sinceTO;
};

UEApp:: UEApp () {}

UEApp:: ~UEApp() {
	m_socketReceiver = 0;
	m_socketSender = 0;
}

void
UEApp::Setup(Address addressSend, Address addressReceive, uint32_t packetSize, uint8_t ueId,  DataRate dataRate,  uint16_t ulPort, uint16_t dlPort, Time decisionPeriod,
		float positiveReward, float negativeReward, double alphaW, uint16_t hiddenSize, std::vector<double> alphaTheta, double gamma, TCPControl tcpCtrl, bool isPeriodic, uint16_t period, uint16_t uniqueID, Timeout rto){
	m_packetSize = packetSize;
	m_serverAddressSend = addressSend;
	m_serverAddressReceive = addressReceive;
	m_ueId = ueId;
	m_dataRate  = dataRate;
	m_ulPort = ulPort;
	m_dlPort = dlPort;
	m_decisionPeriod = decisionPeriod;
	m_positiveReward = positiveReward;
	m_negativeReward = negativeReward;
	m_alphaW = alphaW;
	m_hiddenSize= hiddenSize;
	m_alphaTheta = alphaTheta;
	m_gamma = gamma;
	m_tcpCtrl = tcpCtrl;
	m_isPeriodic = isPeriodic;
	m_period = period;
	m_uniqueID = uniqueID;
	m_to = rto;
}

void
UEApp::StartApplication (void)
{
  std::cout<< " Starting UE Application" << std::endl;
  m_estimatedRTT = 0;
  m_devRTT = 1e2;
  m_running = true;
  m_hasCalledNOAck = false;
  m_isCongested = false;
  m_countRx = 0;
  m_countTx = 0;
  m_cwnd = 1;
  m_cwndMax = 1;
  if(m_tcpCtrl == TCPRL){
	  rl = new RLCompute(m_alphaW, m_hiddenSize, m_alphaTheta, m_gamma, m_ueId);
  }
  else if(m_tcpCtrl == TCPReno){
	  reno = new RenoCompute();
  }
  else if(m_tcpCtrl == TCPCubic){
	  cubic = new CubicCompute();
	  cubic->Setup();
  }
  else{
	  std::cerr<< "Fatal Error: invalid TCP control!"<<std::endl;
	  exit(-1);
  }
  m_ssMode = true;
  m_ssThreshRL = 1e6;
  m_RxcwndCounter = 0;
  m_cwndCounter = m_cwnd;
  m_averageReward = 0;
  m_minRTT = 1e10; // set to arbitrary large value

  m_isOn = true;
  m_clockCounter = 0;
  m_sinceTO = Simulator::Now();
  m_socketSender = Socket::CreateSocket (this->GetNode(), UdpSocketFactory::GetTypeId ());
  m_socketSender->Bind();
  m_socketSender->Connect (m_serverAddressSend);
  m_socketSender->ShutdownRecv();

  m_socketReceiver = Socket::CreateSocket(this->GetNode(), UdpSocketFactory::GetTypeId());
  m_socketReceiver->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_dlPort));
  m_socketReceiver->ShutdownSend();
  m_socketReceiver->SetRecvCallback(MakeCallback(&UEApp::handleRead, this));
  SendPacket();
  std::cout<< "UE Application #" << m_uniqueID << " starts successfully" << std::endl;
 }

void
UEApp::SendPacket(){
	if(m_isPeriodic && !m_isOn){
		return;
	}
	if(m_cwndCounter == 0 && !m_hasCalledNOAck) {
		ScheduleTxNoACK();
		return;
	}
	if(m_hasCalledNOAck) return;
	/*if(m_cwndCounter == 0){
		ScheduleTx();
		return;
	}*/
	Ptr<Packet> packet = Create<Packet> (m_packetSize);
	CustomECNTag ecnTag;
	CustomTCPTag tcpTag;
	TimestampTag timeTag;

	ecnTag.setBits(2);
	tcpTag.setId(m_ueId);
	timeTag.SetTimestamp(Simulator::Now());

	packet->AddPacketTag(ecnTag);
	packet->AddPacketTag(tcpTag);
	packet->AddPacketTag(timeTag);

	m_socketSender->Send(packet);
	++m_countTx;
	--m_cwndCounter;
	ScheduleTx();
}

void
UEApp::SendPacketTimeout (){
	if(m_tcpCtrl == TCPRL){
		m_cwnd = m_cwnd*0.7;
		m_ssThreshRL = m_cwnd;
		if(readCWND && m_uniqueID == observationId){
						std::cout << "cwnd becomes halved due to timeout" << std::endl;
			}
	}
	m_cwndCounter = m_cwnd;
	 m_RxcwndCounter = 0;
	 if(m_tcpCtrl == TCPReno) m_cwnd = reno->retransmit(m_cwnd);
	 if(m_tcpCtrl == TCPCubic) cubic->OnPacketLoss(m_cwnd);
	Ptr<Packet> packet = Create<Packet> (m_packetSize);

	CustomECNTag ecnTag;
	CustomTCPTag tcpTag;
	TimestampTag timeTag;

	ecnTag.setBits(2);
	tcpTag.setId(m_ueId);
	timeTag.SetTimestamp(Simulator::Now());

	packet->AddPacketTag(ecnTag);
	packet->AddPacketTag(tcpTag);
	packet->AddPacketTag(timeTag);
	m_socketSender->Send(packet);
	m_countTx++;
	--m_cwndCounter;
	m_hasCalledNOAck = false;
	ScheduleTx();
}

void
UEApp::ScheduleTx (void){
	if (m_running)
	    {
	      Time tNext (1e9*m_packetSize * 8 / (static_cast<double> (m_dataRate.GetBitRate ())));
	      m_sendEvent = Simulator::Schedule (tNext, &UEApp::SendPacket, this);

	    }
}

void
UEApp::ScheduleTxNoACK (void){
	Time tSchedule;
	if(m_to.time> Time(0)){
		tSchedule = m_to.time;
	}
	else{
		tSchedule = Time((1.1*m_estimatedRTT + 8*m_devRTT)*1e6); //Original 4*
	}
	m_timeoutEventId = Simulator::Schedule(tSchedule, &UEApp::SendPacketTimeout, this);
	m_hasCalledNOAck = true;
}

void
UEApp::StopApplication(void){
	m_running = false;

	if (m_sendEvent.IsRunning ())
	{
	  Simulator::Cancel (m_sendEvent);
	}
	m_socketSender->Close();
	m_socketReceiver->Close();
}
void
UEApp::handleRead(Ptr<Socket> socket){
	if(m_cwnd > m_cwndMax) m_cwndMax = m_cwnd;
	Ptr<Packet> packet;
	Address sender;
	while((packet = socket->RecvFrom(sender))){
		 CustomECNTag tag;
		 TimestampTag timeTag;
		 bool found = packet->PeekPacketTag(tag);
		 bool foundTime = packet->PeekPacketTag(timeTag);
		 if(!found){
			 std::cerr<<"Fatal Error: the ECN tag is not detected on UE side!"<<std::endl;
			 exit(-1);
		 }
		 if(!foundTime){
			 std::cerr<<"Fatal Error: the time tag is not detected on UE side!"<<std::endl;
			 exit(-1);
		 }
		  if(!tag.isECNCapable()){
				std::cout<<"Not ECN capable tag detected"<<std::endl;
			}
			else if(tag.isCongested() || m_isCongested){
				//std::cout << "Congestion event with id = " << m_uniqueID << std::endl;
				m_isCongested = true;
			}
			else if(tag.isPositiveReward()){
				m_isCongested = false;
				m_countRx += 1;
				m_averageReward  +=(m_positiveReward - m_averageReward)/m_countRx;
			}
			else{
				m_isCongested = false;
				m_countRx += 1;
				m_averageReward  +=(m_negativeReward - m_averageReward)/m_countRx;
			}
		  Time current_time = Simulator::Now();
		  Time send_time = timeTag.GetTimestamp();
		  Time RTT = current_time - send_time;
		  if(m_estimatedRTT <= 0.00001){
			  m_estimatedRTT = RTT.GetMilliSeconds();
		  }
		  else{
			  m_estimatedRTT = 0.875*m_estimatedRTT + 0.125*RTT.GetMilliSeconds(); //RFC 6298
			  if(m_estimatedRTT < m_minRTT ){
				  m_minRTT = m_estimatedRTT;
				  //std::cout<<m_minRTT<<" ms"<<std::endl;
			  }
			  m_devRTT = 0.75*m_devRTT + 0.25*std::abs(RTT.GetMilliSeconds() - m_estimatedRTT);
		  }
		  //std::cout<<"Received RTT =" << RTT.GetMilliSeconds()<< " ms"<<std::endl;
		  //std::cout<<"estimated RTT = " << m_estimatedRTT<< " ms" << std::endl;
		 //std::cout<<"dev RTT = " << m_devRTT <<" ms" << std::endl;
		  m_RxcwndCounter++;
		  //std::cout<<"counter: "<<m_RxcwndCounter<<std::endl;
		  if(m_RxcwndCounter == m_cwnd){
			 if(m_hasCalledNOAck) m_timeoutEventId.Cancel();
			  updateReward();
			  m_cwndCounter = m_cwnd;
			  m_RxcwndCounter = 0;
			  m_hasCalledNOAck = false;
			  SendPacket();
		  }
	}
}

double
UEApp::getRTT(){
	return m_estimatedRTT;
}

void
UEApp::updateReward(){
	if(readCWND && m_uniqueID== observationId){
		std::cout<<"Current CWND = " << m_cwnd << std::endl;
		std::cout<<"Current SS RL threshold = " << m_ssThreshRL << std::endl;
	}
	if(m_uniqueID== 0){
			trackCWND.push_back(std::make_pair(Simulator::Now(), m_cwnd));
		}
	if(m_uniqueID == 1){
			trackCWND2.push_back(std::make_pair(Simulator::Now(), m_cwnd));
		}
	if(m_uniqueID == 2){
				trackCWND3.push_back(std::make_pair(Simulator::Now(), m_cwnd));
			}
	if(m_isPeriodic){
		if(m_isOn && (m_clockCounter != m_period)){
			m_clockCounter++;
				//Do nothing
		}
		else if(!m_isOn && (m_clockCounter != m_period)){
			m_clockCounter++;
			Simulator::Schedule (m_decisionPeriod, &UEApp::updateReward, this);
			return;
		}
		else if(m_isOn && (m_clockCounter == m_period)){
			m_isOn = false;
			m_clockCounter = 0;
			m_countRx = 0;
			m_countTx = 0;
			m_cwnd = 0;
			Simulator::Schedule (m_decisionPeriod, &UEApp::updateReward, this);
			return;
		}
		else if(!m_isOn && (m_clockCounter == m_period)){
				m_isOn = true;
				m_clockCounter = 0;
				m_cwnd = 10;
				m_cwndMax = 10;
				m_ssMode = true;
				m_averageReward = 0;
				m_isCongested = false;
				m_estimatedRTT = 0;
				m_devRTT = 1e2;
				m_ssThreshRL = 1e6;
				m_RxcwndCounter = 0;
				m_hasCalledNOAck = false;
				m_cwndCounter = m_cwnd;
				m_minRTT = 1e10;
				SendPacket();
				Simulator::Schedule (m_decisionPeriod, &UEApp::updateReward, this);
				if(m_tcpCtrl == TCPRL) rl->resetPolicyParams();
				if(m_tcpCtrl == TCPCubic) cubic->CubicReset();
				return;
		}

	}

	State curr_state;
	if(m_countRx == 0){
		m_averageReward = m_negativeReward;
	}
	if(m_countTx != 0){
		curr_state.ACKRatio = static_cast<double>(m_countRx)/m_countTx ;
	}
	else{
		curr_state.ACKRatio = 0;
		m_averageReward =  m_negativeReward;
	}
	if(m_minRTT != 0){
		curr_state.RTTRatio = m_estimatedRTT/m_minRTT;
	}
	else{
		curr_state.RTTRatio = 1;
		m_averageReward = m_negativeReward;
	}
	curr_state.cwndRatio = static_cast<double>(m_cwnd)/static_cast<double>(m_cwndMax);
	//trackState.push_back(std::make_pair(curr_state.RTTRatio, curr_state.ACKRatio));
	if(readReward == m_ueId){
		std::cout<<"Get average reward = " << m_averageReward << std::endl;
		if(m_countTx != 0){
		std::cout<<"ACK ratio =  " << curr_state.ACKRatio << std::endl;
		}
		else{
			std::cout<<"count TX is zero"<<std::endl;
		}
		if(m_minRTT != 0)
		std::cout<<"RTT ratio = " << curr_state.RTTRatio << std::endl;
		else
			std::cout<<"m_minRTT is zero"<<std::endl;
	}
	int change;
	if(m_isCongested){
		if(m_tcpCtrl == TCPRL){
			curr_state.isTerminal = true;
			m_ssThreshRL = m_cwnd*0.7;
			if(m_cwnd > 3){
				m_cwnd = m_cwnd *0.7;
					}
			else{
				m_cwnd = 1;
					}
			if(readCWND && m_uniqueID == observationId){
				std::cout << "cwnd becomes halved due to congestion" << std::endl;
			}
			m_ssMode = false;
			rl->compute(m_averageReward, curr_state, m_cwndMax);
		}
		else if(m_tcpCtrl == TCPReno){
			m_cwnd = reno->retransmit(m_cwnd);
		}
	}
	else{
		if(m_tcpCtrl == TCPRL){
			curr_state.isTerminal = false;
			if(m_cwnd > m_ssThreshRL){
				m_ssMode = false;
			}
			/*else{
				m_ssMode = false;
			}*/
			if(m_ssMode){
				m_cwnd = 2*m_cwnd;
			}
			else{
				change = rl->compute(m_averageReward, curr_state, m_cwndMax);
				if(m_cwnd + change <= m_ssThreshRL){
					//std::cout << "Because " << m_cwnd + change << " <= " << m_ssThreshRL << std::endl;
					change = 0;
				}

				int test_cwnd = m_cwnd + change;
				if(test_cwnd > 1){
					m_cwnd += change;
				}
			}
		}
		else if(m_tcpCtrl == TCPReno){
			m_cwnd = reno->update(m_cwnd, m_countTx);
		}
		else{
			m_cwnd = cubic->OnData(m_minRTT, m_cwnd, m_estimatedRTT);
		}
	}
	if(m_cwnd < 1){
				m_cwnd = 1;
		}
	if(readCWND && m_uniqueID == observationId){
		std::cout<<"Change of CWND = " << change << std::endl;
	}
	/*if(m_cwnd < m_cwndMax/4 && m_tcpCtrl == TCPRL){
		m_ssMode = true;
		m_cwndMax = 3*m_cwndMax/4;
		m_ssThreshRL = 3*m_ssThreshRL/4;
	}*/
	m_isCongested = false;
	m_countRx= 0;
	m_countTx = 0;
	m_averageReward = 0;

}

//----------------------------------------- UEApp class definition ends here --------------------------------------------------------

//----------------------------------------- ServerApp class definition starts here -------------------------------------------------
class ServerApp : public Application{
public:
	ServerApp();
	virtual ~ServerApp();
	void Setup(Address enb, uint32_t serverPort);

protected:
	void handleRead(Ptr<Socket> socket);
	void handleAccept(Ptr<Socket> socket, const Address& from);

private:
	virtual void StartApplication (void);
	virtual void StopApplication (void);

	bool m_running;
	Ptr<Socket>    m_socketRecv;
	Ptr<Socket>  m_socketSend;
	Address m_eNBAddress;
    uint32_t m_serverPort;
};

ServerApp::ServerApp(){

}

ServerApp::~ServerApp(){

}

void
ServerApp::Setup(Address enb,  uint32_t serverPort){
	m_eNBAddress = enb;
	m_serverPort = serverPort;
}
void
ServerApp::StartApplication (void)
{
  std::cout<< " Starting Server Application" << std::endl;
  m_running = true;

  m_socketSend= Socket::CreateSocket(this->GetNode(), UdpSocketFactory::GetTypeId());
  m_socketSend->Bind();
  m_socketSend->Connect(m_eNBAddress);
  m_socketSend->ShutdownRecv();


 m_socketRecv= Socket::CreateSocket(this->GetNode(), UdpSocketFactory::GetTypeId());
 m_socketRecv->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_serverPort));
 m_socketRecv->ShutdownSend();
 m_socketRecv->SetRecvCallback(MakeCallback(&ServerApp::handleRead, this));

 }

void
ServerApp::StopApplication(void){
	m_running = false;
	m_socketRecv->Close();
	m_socketSend->Close();
}

void
ServerApp::handleRead(Ptr<Socket> socket){
	if(!m_running) {return;}
	Ptr<Packet> packet;
	Address sender;
	while((packet = socket->RecvFrom(sender))){
		m_socketSend->Send(packet);
	}

}
//------------------------------------ ServerApp class definition ends here -------------------------------------------------------------------

int
main (int argc, char *argv[])
{
		RngSeedManager::SetSeed(23);

		//For debugging purpose
		observationId = 2; //Read which UE's policy and/or UE's cwnd

		readThroughput = 2; //Read which UE's queue util
		whicheNB = 0;
		verboseTr = false;

		readQueue = false;
		readReward = 3;
		readCWND = true;
		readPolicy = true;
		readBias = false;

		uint16_t ulPort = 10000;  //Uplink port offset for eNB0
		uint16_t dlPort = 20000;  //Downlink port offset for eNB0
		uint16_t serverPortUl = 30000;
		uint16_t serverPortDl = 40000;
		Time startTime = Seconds(0.03);  //start application time
		Time simTime = Seconds(160); //simulation time
		uint16_t packetSize = 1500; //size of 1 MTU in bytes
		Timeout rto = {MilliSeconds(0)}; //0 means using RFC standard

		double errorRate = 5e-7;  //Dropping rate due to receiving side of eNB 0 and UE 0
		double errorRate2 = 5e-7;  //Dropping rate due to receiving side of server and eNB 0
		double errorRate3 = 5e-7;  //Dropping rate due to receiving side of eNB 0 and UE 1
		double errorRate4 = 5e-7;  //Dropping rate due to receiving side of UE 2 and eNB 1

		DataRate dataRate = DataRate("8Mbps");  //Base rate for UE1 Dont set too high
		DataRate dataRate2 = DataRate("8Mbps");  //Base rate for UE 2 Dont set too high
		DataRate dataRate3 = DataRate("8Mbps");  //Base rate for UE 3 Dont set too high

		//Parameters for queue

		//eNB 0
		uint16_t queueSize= 300;
		Time queueDelay = MicroSeconds(400); //500

		float eta = 0.8;  //Relative low throughput tolerance
		double dropFullProbability = 0.95; //When the queue above the threshold(or queueSize) 0.95
		float posReward = 7;
		float negReward = -1;
		Time decisionTime = MilliSeconds(20);
		double alphaW = 5e-4; //5e-4
		uint32_t hiddenSize = 10; //7
		double alphaThetaArr[3] ={7e-4, 7e-4, 7e-4};
		std::vector<double> alphaTheta = std::vector<double>(alphaThetaArr, alphaThetaArr + 3);
		double gamma = 0.9;
		TCPControl tcp1 = TCPRL;
		TCPControl tcp2 = TCPRL;
		TCPControl tcp3 = TCPRL;
		bool isExp4 = false;
		uint16_t period = 200; //On off period

		NodeContainer nodes;
		nodes.Create (5); //index 0 is UE 0, index 1 is eNB 0, index 2 is server, index 3 is UE 1 index 4 is UE 2

		PointToPointHelper pointToPoint; //Between UE0 and eNB 0
		pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("1Gbps"));
		pointToPoint.SetChannelAttribute ("Delay", StringValue ("1ms"));
		pointToPoint.SetQueue ("ns3::DropTailQueue", "MaxSize", StringValue ("80000000p"));

		PointToPointHelper p2pserver; //Between eNB 0 and server
		p2pserver.SetDeviceAttribute ("DataRate", StringValue ("1Gbps"));
		p2pserver.SetChannelAttribute ("Delay", StringValue ("3ms"));
		p2pserver.SetQueue ("ns3::DropTailQueue", "MaxSize", StringValue ("80000000p"));

		PointToPointHelper p2pue2; //Between UE 1 and eNB 0
		p2pue2.SetDeviceAttribute ("DataRate", StringValue ("1Gbps"));
		p2pue2.SetChannelAttribute ("Delay", StringValue ("1ms"));
		p2pue2.SetQueue ("ns3::DropTailQueue", "MaxSize", StringValue ("80000000p"));


		PointToPointHelper p2ue3; //Between UE 2 and eNB 0
		p2ue3.SetDeviceAttribute ("DataRate", StringValue ("1Gbps"));
		p2ue3.SetChannelAttribute ("Delay", StringValue ("1ms"));
		p2ue3.SetQueue ("ns3::DropTailQueue", "MaxSize", StringValue ("80000000p"));


		NetDeviceContainer devicesUeEnb;
		devicesUeEnb = pointToPoint.Install (nodes.Get(0), nodes.Get(1));

		NetDeviceContainer devicesEnbServer;
		devicesEnbServer = p2pserver.Install(nodes.Get(1), nodes.Get(2));

		NetDeviceContainer devicesUeEnb2;
		devicesUeEnb2 = p2pue2.Install(nodes.Get(3), nodes.Get(1));

		NetDeviceContainer devicesUeEnb3;
		devicesUeEnb3 = p2ue3.Install(nodes.Get(4), nodes.Get(1));

		InternetStackHelper stack;
		stack.Install (nodes);

		Ptr<RateErrorModel> em = CreateObject<RateErrorModel> (); //Error model for device 1 -> Enb
		em->SetAttribute ("ErrorRate", DoubleValue (errorRate));
		devicesUeEnb.Get (0)->SetAttribute ("ReceiveErrorModel", PointerValue (em));
		devicesUeEnb.Get (1)->SetAttribute ("ReceiveErrorModel", PointerValue (em));

		Ptr<RateErrorModel> em2 = CreateObject<RateErrorModel> (); //Error model for Enb -> server
		em2->SetAttribute("ErrorRate", DoubleValue(errorRate2));
		devicesEnbServer.Get(1)->SetAttribute("ReceiveErrorModel" ,PointerValue(em2));
		devicesEnbServer.Get(0)->SetAttribute("ReceiveErrorModel" ,PointerValue(em2));

		Ptr<RateErrorModel> em3 = CreateObject<RateErrorModel> (); //Error model for device 2 -> Enb
		em3->SetAttribute("ErrorRate", DoubleValue(errorRate3));
		devicesUeEnb2.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue (em3));
		devicesUeEnb2.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue (em3));

		Ptr<RateErrorModel> em4 = CreateObject<RateErrorModel>();
		em4->SetAttribute("ErrorRate", DoubleValue(errorRate4));
		devicesUeEnb3.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue (em4));
		devicesUeEnb3.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue (em4));

		 Ipv4AddressHelper ipv4;
		 ipv4.SetBase ("10.1.1.0", "255.255.255.0");
		 Ipv4InterfaceContainer interfacesUeEnb = ipv4.Assign (devicesUeEnb);

		 ipv4.SetBase("10.1.2.0", "255.255.255.0");
		 Ipv4InterfaceContainer interfacesUeEnb2 = ipv4.Assign (devicesUeEnb2);

		 ipv4.SetBase("10.1.3.0", "255.255.255.0");
		 Ipv4InterfaceContainer interfacesEnbServer = ipv4.Assign(devicesEnbServer);

		 ipv4.SetBase("10.1.4.0", "255.255.255.0");
		 Ipv4InterfaceContainer interfacesUeEnb3 = ipv4.Assign(devicesUeEnb3);

		 //App for eNB0
		Ptr<EnbApp> app = CreateObject<EnbApp>();
		std::pair <Address, uint8_t> ueenbpair = std::make_pair(InetSocketAddress(interfacesUeEnb.GetAddress (0),dlPort),0);
		std::pair <Address, uint8_t> ueenbpair2 = std::make_pair(InetSocketAddress(interfacesUeEnb2.GetAddress (0),dlPort+1),1);
		std::pair <Address, uint8_t> ueenbpair3 = std::make_pair(InetSocketAddress(interfacesUeEnb3.GetAddress (0),dlPort+2),2);
		std::vector<std::pair <Address, uint8_t>> pairHolder;
		pairHolder.push_back(ueenbpair);
		pairHolder.push_back(ueenbpair2);
		pairHolder.push_back(ueenbpair3);

		app->Setup(pairHolder, ulPort, 80, packetSize, eta, queueDelay, queueSize, dropFullProbability, InetSocketAddress(interfacesEnbServer.GetAddress(1), serverPortDl), serverPortUl, isExp4, period*decisionTime);
		app->SetStartTime (startTime);
		app->SetStopTime (simTime);
		nodes.Get(1)->AddApplication(app);

		//App for UE0
		Ptr<UEApp> app1 = CreateObject<UEApp>();
		app1->Setup(InetSocketAddress(interfacesUeEnb.GetAddress(1), ulPort), InetSocketAddress(interfacesUeEnb.GetAddress(0), dlPort), packetSize, 0,  dataRate, ulPort, dlPort, decisionTime, posReward,
				negReward, alphaW, hiddenSize, alphaTheta, gamma, tcp1, false, 0, 0, rto);
		app1->SetStartTime (startTime);
		app1->SetStopTime (simTime);
		nodes.Get(0)->AddApplication(app1);

		//App for UE1
		Ptr<UEApp> app2 = CreateObject<UEApp>();
		app2->Setup(InetSocketAddress(interfacesUeEnb2.GetAddress(1), ulPort+1), InetSocketAddress(interfacesUeEnb2.GetAddress(0), dlPort), packetSize, 1,  dataRate2, ulPort, dlPort+1, decisionTime, posReward,
				negReward, alphaW, hiddenSize, alphaTheta, gamma, tcp2, isExp4, period, 1, rto);
		app2->SetStartTime(startTime);
		app2->SetStopTime(simTime);
		nodes.Get(3)->AddApplication(app2);

		//App for UE2
		Ptr<UEApp> app4 = CreateObject<UEApp>();
		app4->Setup(InetSocketAddress(interfacesUeEnb3.GetAddress(1), ulPort + 2), InetSocketAddress(interfacesUeEnb3.GetAddress(0), dlPort), packetSize, 2,  dataRate3, ulPort, dlPort + 2, decisionTime, posReward,
						negReward, alphaW, hiddenSize, alphaTheta, gamma, tcp3, false, 0, 2, rto);
		app4->SetStartTime(startTime);
		app4->SetStopTime(simTime);
		nodes.Get(4)->AddApplication(app4);

		Ptr<ServerApp> serverApp = CreateObject<ServerApp>();
		serverApp->Setup(InetSocketAddress(interfacesEnbServer.GetAddress(0), serverPortUl),  serverPortDl);
		serverApp->SetStartTime(startTime);
		serverApp->SetStopTime(simTime);
		nodes.Get(2)->AddApplication(serverApp);

		Simulator::Stop (simTime);
		Simulator::Run ();
		Simulator::Destroy ();
		 std::string fileNameWithNoExtension = "cwnd-plot";
		 std::string graphicsFileName  = fileNameWithNoExtension + ".png";
		 std::string plotFileName  = fileNameWithNoExtension + ".plt";

		 Gnuplot plot (graphicsFileName);
		 plot.SetTitle ("CWND trend");
		 plot.SetTerminal ("png");
		 plot.SetLegend ("time (ms)", "CWND");
		 plot.AppendExtra ("set yrange [0:+500]");
		 Gnuplot2dDataset dataset;
		 Gnuplot2dDataset ds;
		 Gnuplot2dDataset datas;
		 dataset.SetTitle("UE 0");
		 dataset.SetStyle (Gnuplot2dDataset::LINES);
		 ds.SetTitle("UE 1");
		 ds.SetStyle (Gnuplot2dDataset::LINES);
		 datas.SetTitle("UE 2");
		 datas.SetStyle (Gnuplot2dDataset::LINES);
		 Time counter_time = decisionTime;
		 uint32_t count_cwnd = 0;
		 uint32_t count_cwnd4 = 0;
		 double cwnd1_avg = 0;
		 double cwnd2_avg = 0;
		 double cwnd3_avg = 0;
		 double cwnd1_avgOn = 0;
		 double cwnd2_avgOn = 0;
		 for(uint32_t i = 0; i < trackCWND.size(); ++i){
			 count_cwnd++;
			 dataset.Add(trackCWND[i].first.GetMilliSeconds(), trackCWND[i].second);
			 cwnd1_avg += (trackCWND[i].second - cwnd1_avg)/count_cwnd;
		 }
		 count_cwnd = 0;

		 for(uint32_t i = 0; i < trackCWND2.size(); ++i){
			 count_cwnd++;
			 datas.Add(trackCWND2[i].first.GetMilliSeconds(), trackCWND2[i].second);
			 cwnd2_avg += (trackCWND2[i].second - cwnd2_avg)/count_cwnd;
		 }
		 count_cwnd = 0;
		 for(uint32_t i = 0; i < trackCWND3.size(); ++i){
					 count_cwnd++;
					 ds.Add(trackCWND3[i].first.GetMilliSeconds(), trackCWND3[i].second);
					 cwnd3_avg += (trackCWND3[i].second - cwnd3_avg)/count_cwnd;
				 }
		 plot.AddDataset(dataset);
		 plot.AddDataset(ds);
		 plot.AddDataset(datas);
		 std::ofstream plotFile (plotFileName.c_str());
		 plot.GenerateOutput (plotFile);
		 plotFile.close ();

		 fileNameWithNoExtension = "throughput-plot";
		 graphicsFileName  = fileNameWithNoExtension + ".png";
		 plotFileName  = fileNameWithNoExtension + ".plt";
		 Gnuplot plot2 (graphicsFileName);
		 plot2.SetTitle ("Queue Validity");
		 plot2.SetTerminal ("png");
		 plot2.SetLegend ("time (ms)", "Num of packets inside the queue");
		 plot2.AppendExtra ("set yrange [0:+250]");
		 //plot.AppendExtra ("set yrange [0:+1000]");
		 Gnuplot2dDataset dataset2;
		 Gnuplot2dDataset dataset3;
		 Gnuplot2dDataset dataset4;
		 dataset2.SetTitle("Queue Allocation");
		 dataset3.SetTitle("Upper bound");
		 dataset4.SetTitle("Lower bound");
		 dataset2.SetStyle (Gnuplot2dDataset::LINES);
		 dataset3.SetStyle (Gnuplot2dDataset::LINES);
		 dataset4.SetStyle (Gnuplot2dDataset::LINES);
		 Time movingTime = trackCalculatedThroughput[0].first;
		 Time basedTime = trackCalculatedThroughput[0].first;

		 double moving_average = 0;
		 uint32_t count = 0;
		 for(uint32_t i = 1; i < trackCalculatedThroughput.size(); ++i){
			 movingTime = trackCalculatedThroughput[i].first;
			 if(movingTime - basedTime < 5*decisionTime){
				 count++;
				 moving_average += (trackCalculatedThroughput[i].second - moving_average)/count;
			 }
			 else{
				 dataset2.Add(trackCalculatedThroughput[i].first.GetMilliSeconds(), moving_average);
				 dataset3.Add(trackCalculatedThroughput[i].first.GetMilliSeconds(),trackExpectedThroughput);
				 dataset4.Add(trackCalculatedThroughput[i].first.GetMilliSeconds(),eta*trackExpectedThroughput);
				 moving_average = 0;
				 count = 0;
				 basedTime = movingTime;
			 }
		 }
		 plot2.AddDataset(dataset2);
		 plot2.AddDataset(dataset3);
		 plot2.AddDataset(dataset4);
		 std::ofstream plotFile2 (plotFileName.c_str());
		 plot2.GenerateOutput (plotFile2);
		 plotFile2.close ();

		 /*fileNameWithNoExtension = "state-scatter";
		 graphicsFileName  = fileNameWithNoExtension + ".png";
		 plotFileName  = fileNameWithNoExtension + ".plt";
		 Gnuplot plotS (graphicsFileName);
		 plotS.SetTitle ("State scatter");
		 plotS.SetTerminal ("png");
		 plotS.SetLegend ("RTT ratio", "ACK ratio");
		 Gnuplot2dDataset scatterSet;
		 scatterSet.SetTitle("States");
		 scatterSet.SetStyle (Gnuplot2dDataset::DOTS);
		 for(uint32_t i = 0; i < trackState.size(); ++i){
			 scatterSet.Add(trackState[i].first,trackState[i].second);
		 }std::cout<<"Utilization of UE 1 = "<< ((cwnd2_avg*packetSize*8)/(app2->getRTT()*dataRate2.GetBitRate()*1e3)) <<std::endl;
		 plotS.AddDataset(scatterSet);
		 std::ofstream plotFilesc (plotFileName.c_str());
		 plotS.GenerateOutput (plotFilesc);
		 plotFilesc.close ();*/
		 double queue_avg = 0;
		 double queue_var = 0;
		 uint32_t queueMax = 0;
		 uint32_t count_queue = 0;
		 for(uint32_t i = 0; i < trackQueueSize.size(); ++i){
			 count_queue++;
			 queue_avg += (trackQueueSize[i] - queue_avg)/count_queue;
			 queue_var += (trackQueueSize[i]*trackQueueSize[i] - queue_var)/count_queue;
			 queueMax = std::max(queueMax,trackQueueSize[i]);
		 }
		 queue_var -= queue_avg*queue_avg;

		 std::cout<<"Average queue load = "<<queue_avg<<std::endl;
		 std::cout<<"Variance of queue load = "<<queue_var<<std::endl;
		 std::cout<<"Maximum queue load = "<<queueMax<<std::endl;
		 if(isExp4){
			 double fairness = std::abs(cwnd1_avgOn - cwnd2_avgOn)*100/std::max(cwnd1_avgOn,cwnd2_avgOn);
			 std::cout<<"Fairness metric = " << fairness << " %"<<std::endl;
			 std::cout<<"cwnd avg of UE 0 when UE 1 is on = " << (cwnd1_avgOn*dataRate.GetBitRate()/1e6)<< " Mbps"<<std::endl;
			 double avg_cwnd = (cwnd1_avg*count_cwnd - cwnd1_avgOn*count_cwnd4)/(count_cwnd - count_cwnd4);
			 std::cout<<"cwnd avg of UE 0 when UE 1 is off = " << (avg_cwnd*dataRate.GetBitRate()/1e6) << " Mbps"<<std::endl;
			 std::cout<<"cwnd avg of UE 1 when on = " << (cwnd2_avgOn*dataRate2.GetBitRate()/1e6)  << " Mbps"<<std::endl;
		 }
		 else{
			 std::cout<<"Utilization of UE 0 = "<< ((cwnd1_avg*packetSize*8)/(app1->getRTT()*dataRate.GetBitRate()*1e3))  <<std::endl;
			 std::cout<<"Utilization of UE 1 = "<< ((cwnd2_avg*packetSize*8)/(app2->getRTT()*dataRate2.GetBitRate()*1e3)) <<std::endl;
			 std::cout<<"Utilization of UE 2 = "<< ((cwnd3_avg*packetSize*8)/(app4->getRTT()*dataRate3.GetBitRate()*1e3)) <<std::endl;
			 std::cout<< "Average RTT for UE 0 = " << app1->getRTT() <<" ms" <<  std::endl;
			 std::cout<<"Average RTT for UE 1 = " << app2->getRTT() << " ms" << std::endl;
			 std::cout<<"Average RTT for UE 2 = " << app4->getRTT() << " ms" << std::endl;
			 //Use Jain's fairness index with 1/3 being the worst and 1 being the best
			 double queue0Avg = 1.0*std::accumulate(queue1.begin(), queue1.end(), 0LL)/queue1.size();
			 double queue1Avg = 1.0*std::accumulate(queue2.begin(), queue2.end(), 0LL)/queue2.size();
			 double queue2Avg = 1.0*std::accumulate(queue3.begin(), queue3.end(), 0LL)/queue3.size();
			 double fairness = std::pow((queue0Avg + queue1Avg + queue2Avg),2)/(3*(queue0Avg*queue0Avg + queue1Avg*queue1Avg +queue2Avg*queue2Avg ));
			 std::cout<<"Fairness metric = " << fairness <<std::endl;
		 }
		 return 0;
}





