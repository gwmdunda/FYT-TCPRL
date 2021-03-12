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

std::vector<uint32_t> trackCWND;
std::vector<uint32_t> trackCWND2;
std::vector<uint32_t> trackQueueSize;
double trackExpectedThroughput;
std::vector<std::pair<Time, double>> trackCalculatedThroughput;
std::vector<std::pair<double, double>> trackState;
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
			float m_eta, Time timeDelay, uint16_t maxQueue, float dropP, Address server, uint16_t m_serverPort, bool isExp4, Time period, float chi);

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
    float m_expectedThroughput; //Should be the same across the users to ensure fairness
    std::vector<float> accumulatedBytes;
    std::vector<double >current_throughput;
    Address m_server;
    Ptr<Socket> m_socketToServer;
    Ptr<Socket> m_socketFromServer;
    uint16_t m_serverPort; //Uplink server port
    bool m_isExp4;
    Time m_period;
    bool m_exp4State;
    float m_chi;
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
		float eta, Time timeDelay, uint16_t maxQueue, float dropP, Address server, uint16_t serverPort, bool isExp4, Time period, float chi)
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
  m_chi = chi;
}

void
EnbApp::setExpectedThroughput(bool isComingUE){

	m_expectedThroughput = m_chi*m_packetSize*4/(m_serviceTime.GetMicroSeconds()) ;

	if(isComingUE){
		m_numberOfUE++;
		m_expectedThroughput = m_expectedThroughput /m_numberOfUE;
		accumulatedBytes.push_back(0);
		current_throughput.push_back(0);
	}
	else{
		m_numberOfUE--;
		//TODO: handle deletion by id of USER, only worry this part when we are in handover stage
		if(m_numberOfUE != 0){
			m_expectedThroughput = m_expectedThroughput/m_numberOfUE;
		}
	}
	trackExpectedThroughput = m_expectedThroughput;
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
		if(m_queueCounter > m_maxQueueSize){
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
	if(readQueue){
		//if(m_queueCounter > 60)
		std::cout<<"Current scheduler queue size: "<<m_queueCounter<<std::endl;
	}
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
		accumulatedBytes[ueId] += packet->GetSize();
		Time current_time = Simulator::Now();

		if(m_queueCounter > m_maxQueueSize){
			 double val = sample->GetValue();
			 if(val < m_dropProbability) {
				 --m_queueCounter;
				 return;}
			 uint16_t maxIndex = max_element(current_throughput.begin(), current_throughput.end()) - current_throughput.begin();
			 if(current_throughput[ueId] >m_expectedThroughput && maxIndex == ueId){
				 ecnTag.setBits(3); //Congested State
			 }
			Simulator::Schedule (m_queueCounter*m_serviceTime, &EnbApp::SendPacketToUE, this, ueId, ecnTag, timeTag);
			return;
		}

		if(current_time - m_lastReceivedTimes[ueId]> m_serviceTime){ //Update measured throughput for each service time
			current_throughput[ueId] = 0.6*current_throughput[ueId] + 0.4*accumulatedBytes[ueId]*8/(current_time - m_lastReceivedTimes[ueId]).GetMicroSeconds();
			accumulatedBytes[ueId] = 0;
			m_lastReceivedTimes[ueId] = current_time;
		}
		if(ueId == 0){
			trackCalculatedThroughput.push_back(std::make_pair(current_time, current_throughput[0]));
		}
		if(readThroughput == ueId){
			std::cout<<"Current throughput for user  "<<(int)ueId<<"is : "<<current_throughput[ueId] << std::endl;
			std::cout<<"Compared throughput:"<<m_expectedThroughput<<std::endl;
		}
		if(ecnTag.isPositiveReward()){
			if(current_throughput[ueId] > m_expectedThroughput|| current_throughput[ueId] < (m_eta*m_expectedThroughput)){
				//Send to UE directly
				ecnTag.setBits(1); //You are penalized!
			}
		}
		Simulator::Schedule (m_queueCounter*m_serviceTime, &EnbApp::SendPacketToServer, this, ueId, ecnTag, timeTag);

	}

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

		sampleX->SetAttribute("Min", DoubleValue(-0.1));
		sampleX->SetAttribute("Max", DoubleValue(0));
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
				return -floor(sqrt(state.cwndRatio*m_cwndMax));
			case 1:
				return -1;
			case 2:
				return 0;
			case 3:
				return 1;
			case 4:
				return floor(sqrt(state.cwndRatio*m_cwndMax));
		}
		return 0;

}

//------------------------------RLCompute class definition ends here -----------------------------

//------------------------------RenoCompute class definition starts here ------------------------
class RenoCompute{ //Note we do not consider fast recovery since we always assume no dups and always arrive in order
private:
	uint32_t m_ssThresh;
	uint32_t m_initialCwnd;
	uint32_t m_retxThresh;

public:
	void setSSThresh(uint32_t ssThresh);
	uint32_t update(uint32_t cwnd, uint32_t countPacket); //note that this return cwnd instead of its change
	RenoCompute();
	void retransmit(uint32_t cwnd);
};

RenoCompute::RenoCompute(){
	 m_ssThresh = 1e6;
	 m_initialCwnd = 10;
	 m_retxThresh = 3;
}

void
RenoCompute::setSSThresh(uint32_t ssThresh){
	m_ssThresh = ssThresh;
}

uint32_t
RenoCompute::update(uint32_t cwnd, uint32_t countPacket){
	uint16_t new_cwnd;
	if(cwnd < m_ssThresh){ //slow start mode
			new_cwnd = cwnd *2;
	}
	else{ //congestion avoidance mode
		double adder = countPacket/cwnd;
		adder = std::max(1.0,adder);
		new_cwnd = cwnd + static_cast<uint16_t> (adder);
	}
	return new_cwnd;
}

void
RenoCompute::retransmit(uint32_t cwnd){
	std::cout<<"TIMEOUT change ssThresh to = " << m_ssThresh << std::endl;
	m_ssThresh =  cwnd/2;

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
			float positiveReward, float negativeReward, double alphaW, uint16_t hiddenSize, std::vector<double> alphaTheta, double gamma, TCPControl tcpCtrl,  bool isPeriodic, uint16_t period, uint16_t uniqueID);

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
	uint16_t m_ssThreshRL;

	bool m_isOn;
	bool m_isPeriodic;
	uint16_t m_clockCounter;
	uint16_t m_period;
	uint16_t m_uniqueID;
};

UEApp:: UEApp () {}

UEApp:: ~UEApp() {
	m_socketReceiver = 0;
	m_socketSender = 0;
}

void
UEApp::Setup(Address addressSend, Address addressReceive, uint32_t packetSize, uint8_t ueId,  DataRate dataRate,  uint16_t ulPort, uint16_t dlPort, Time decisionPeriod,
		float positiveReward, float negativeReward, double alphaW, uint16_t hiddenSize, std::vector<double> alphaTheta, double gamma, TCPControl tcpCtrl, bool isPeriodic, uint16_t period, uint16_t uniqueID){
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
}

void
UEApp::StartApplication (void)
{
  std::cout<< " Starting UE Application" << std::endl;
  m_estimatedRTT = 0;
  m_devRTT = 0;
  m_running = true;
  m_hasCalledNOAck = false;
  m_isCongested = false;
  m_countRx = 0;
  m_countTx = 0;
  m_cwnd = 10;
  m_cwndMax = 10;
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
  m_ssThreshRL = 655;
  m_RxcwndCounter = 0;
  m_cwndCounter = m_cwnd;
  m_averageReward = 0;
  m_minRTT = 1e10; // set to arbitrary large value

  m_isOn = true;
  m_clockCounter = 0;

  m_socketSender = Socket::CreateSocket (this->GetNode(), UdpSocketFactory::GetTypeId ());
  m_socketSender->Bind();
  m_socketSender->Connect (m_serverAddressSend);
  m_socketSender->ShutdownRecv();

  m_socketReceiver = Socket::CreateSocket(this->GetNode(), UdpSocketFactory::GetTypeId());
  m_socketReceiver->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_dlPort));
  m_socketReceiver->ShutdownSend();
  m_socketReceiver->SetRecvCallback(MakeCallback(&UEApp::handleRead, this));

  Simulator::Schedule (m_decisionPeriod, &UEApp::updateReward, this);
  SendPacket();

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
	if(m_cwndCounter == 0){
		ScheduleTx();
		return;
	}
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
	ScheduleTx();
	--m_cwndCounter;
	m_hasCalledNOAck = false;
}

void
UEApp::SendPacketTimeout (){
	m_cwndCounter = m_cwnd;
	 m_RxcwndCounter = 0;
	 if(m_tcpCtrl == TCPReno) reno->retransmit(m_cwnd);
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

}

void
UEApp::ScheduleTx (void){
	if (m_running)
	    {
	      Time tNext (Seconds (m_packetSize * 8 / (static_cast<double>(m_cwnd)*static_cast<double> (m_dataRate.GetBitRate ()))));
	      m_sendEvent = Simulator::Schedule (tNext, &UEApp::SendPacket, this);
	    }
}

void
UEApp::ScheduleTxNoACK (void){
	 ScheduleTx();
	 Time tSchedule = Time((m_estimatedRTT + 4*m_devRTT)*1e6);

	 //std::cout<<"Scheduling No ack time:"<<tSchedule.GetMilliSeconds()<<" ms"<<std::endl;
	Simulator::Schedule(tSchedule, &UEApp::SendPacketTimeout, this);
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
			  m_cwndCounter = m_cwnd;
			  m_RxcwndCounter = 0;
		  }
	}
}

void
UEApp::updateReward(){
	if(readCWND && m_ueId == observationId){
		std::cout<<"Current CWND = " << m_cwnd << std::endl;
	}
	if(m_uniqueID== 0){
			trackCWND.push_back(m_cwnd);
		}
	if(m_uniqueID == observationId){
			trackCWND2.push_back(m_cwnd);
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
				m_devRTT = 0;
				m_ssThreshRL = 655;
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

	if(m_cwndCounter == 0){
		//Agent must do nothing when timeout
		Simulator::Schedule (m_decisionPeriod, &UEApp::updateReward, this);
		return;
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
			m_ssMode = false;
			rl->compute(m_averageReward, curr_state, m_cwndMax);
		}
		if(m_cwnd > 10){
			m_cwnd = m_cwnd *3/4;
		}
		else{
			m_cwnd = 10;
		}
	}
	else{
		if(m_tcpCtrl == TCPRL){
			curr_state.isTerminal = false;
			if(m_cwnd > m_ssThreshRL){
				m_ssMode = false;
			}
			if(m_ssMode){
				m_cwnd = 2*m_cwnd;
			}
			else{
				change = rl->compute(m_averageReward, curr_state, m_cwndMax);
			}
			int test_cwnd = m_cwnd + change;
			if(test_cwnd > 1){
				m_cwnd += change;
			}
		}
		else if(m_tcpCtrl == TCPReno){
			m_cwnd = reno->update(m_cwnd, m_countTx);
		}
		else{
			m_cwnd = cubic->OnData(m_minRTT, m_cwnd, m_estimatedRTT);
		}
	}
	if(m_cwnd < 10){
				m_cwnd = 10;
		}
	if(readCWND && m_ueId == observationId){
		std::cout<<"Change of CWND = " << change << std::endl;
	}
	if(m_cwnd < m_cwndMax/4 && m_tcpCtrl == TCPRL){
		rl->resetPolicyParams();
		m_ssMode = true;
		m_cwndMax = 3*m_cwndMax/4;
		m_ssThreshRL = 3*m_ssThreshRL/4;
	}
	m_isCongested = false;
	m_countRx= 0;
	m_countTx = 0;
	m_averageReward = 0;
	Simulator::Schedule (m_decisionPeriod, &UEApp::updateReward, this);
}

//----------------------------------------- UEApp class definition ends here --------------------------------------------------------

//----------------------------------------- ServerApp class definition starts here -------------------------------------------------
class ServerApp : public Application{
public:
	ServerApp();
	virtual ~ServerApp();
	void Setup(Address enb, Address enb1,  uint32_t serverPort);

protected:
	void handleRead(Ptr<Socket> socket);
	void handleAccept(Ptr<Socket> socket, const Address& from);
	void handleRead1(Ptr<Socket> socket);
private:
	virtual void StartApplication (void);
	virtual void StopApplication (void);

	bool m_running;
	Ptr<Socket>    m_socketRecv[2];
	Ptr<Socket>  m_socketSend[2];
	Address m_eNBAddress[2];
    uint32_t m_serverPort; //downlink
};

ServerApp::ServerApp(){

}

ServerApp::~ServerApp(){

}

void
ServerApp::Setup(Address enb, Address enb1,  uint32_t serverPort){
	m_eNBAddress[0] = enb;
	m_eNBAddress[1] = enb1;
	m_serverPort = serverPort;
}
void
ServerApp::StartApplication (void)
{
  std::cout<< " Starting Server Application" << std::endl;
  m_running = true;

  m_socketSend[0]= Socket::CreateSocket(this->GetNode(), UdpSocketFactory::GetTypeId());
  m_socketSend[0]->Bind();
  m_socketSend[0]->Connect(m_eNBAddress[0]);
  m_socketSend[0]->ShutdownRecv();

  m_socketSend[1] = Socket::CreateSocket(this->GetNode(), UdpSocketFactory::GetTypeId());
  m_socketSend[1]->Bind();
  m_socketSend[1]->Connect(m_eNBAddress[1]);
  m_socketSend[1]->ShutdownRecv();

 m_socketRecv[0]= Socket::CreateSocket(this->GetNode(), UdpSocketFactory::GetTypeId());
 m_socketRecv[0]->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_serverPort));
 m_socketRecv[0]->ShutdownSend();
 m_socketRecv[0]->SetRecvCallback(MakeCallback(&ServerApp::handleRead, this));

 m_socketRecv[1] = Socket::CreateSocket(this->GetNode(), UdpSocketFactory::GetTypeId());
 m_socketRecv[1]->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_serverPort + 1));
 m_socketRecv[1]->ShutdownSend();
 m_socketRecv[1]->SetRecvCallback(MakeCallback(&ServerApp::handleRead1, this));

 }

void
ServerApp::StopApplication(void){
	m_running = false;
	m_socketRecv[0]->Close();
	m_socketRecv[1]->Close();
	m_socketSend[0]->Close();
	m_socketSend[1]->Close();
}

void
ServerApp::handleRead(Ptr<Socket> socket){
	if(!m_running) {return;}
	Ptr<Packet> packet;
	Address sender;
	while((packet = socket->RecvFrom(sender))){
		m_socketSend[0]->Send(packet);
	}

}

void
ServerApp::handleRead1(Ptr<Socket> socket){
	if(!m_running) {return;}
	Ptr<Packet> packet;
	Address sender;
	while((packet = socket->RecvFrom(sender))){
		m_socketSend[1]->Send(packet);
	}

}
//------------------------------------ ServerApp class definition ends here -------------------------------------------------------------------

int
main (int argc, char *argv[])
{
		RngSeedManager::SetSeed(171);

		observationId = 2; //Read which UE's policy and/or UE's cwnd
		readThroughput = 2;
		readQueue = false;
		readReward = 2;
		readCWND = false;
		readPolicy = false;
		readBias = false;

		uint16_t ulPort = 10000;  //Uplink port offset for eNB0
		uint16_t dlPort = 20000;  //Downlink port offset for eNB0
		uint16_t ulPort1 = 50000;  //Uplink port offset for eNB1
		uint16_t dlPort1 = 60000; //Downlink port offset for eNB1
		uint16_t serverPortUl = 30000;
		uint16_t serverPortDl = 40000;
		Time startTime = Seconds(0.03);  //start application time
		Time simTime = Seconds(15); //simulation time
		uint16_t packetSize = 1500; //size of 1 MTU in bytes

		double errorRate = 0;  //Dropping rate due to receiving side of eNB 0 and UE 0
		double errorRate2 = 0;  //Dropping rate due to receiving side of server and eNB 0
		double errorRate3 = 0;  //Dropping rate due to receiving side of eNB 0 and UE 1
		double errorRate4 = 0;  //Dropping rate due to receiving side of server and eNB 1
		double errorRate5 =0; //Dropping rate due to receiving side of UE 2 and eNB 1

		DataRate dataRate = DataRate("0.1Mbps");  //Base rate for UE1
		DataRate dataRate2 = DataRate("0.1Mbps");  //Base rate for UE 2
		DataRate dataRate3 = DataRate("0.1Mbps");  //Base rate for UE 3

		//Parameters for queue

		//eNB 0
		uint16_t queueSize= 100;
		Time queueDelay = MicroSeconds(100);

		//eNB 1
		uint16_t queueSize1= 100;
		Time queueDelay1 = MicroSeconds(100);

		float eta = 0.9;

		double dropFullProbability = 0.9;
		float posReward = 3;
		float negReward = -1;
		Time decisionTime = MilliSeconds(20);
		double alphaW = 0.005;
		uint32_t hiddenSize = 7;
		double alphaThetaArr[3] ={4e-3, 4e-3, 4e-3};
		std::vector<double> alphaTheta = std::vector<double>(alphaThetaArr, alphaThetaArr + 3);
		double gamma = 0.9;
		TCPControl tcp1 = TCPReno;
		TCPControl tcp2 = TCPReno;
		TCPControl tcp3 = TCPReno;
		bool isExp4 = false;
		uint16_t period = 200; //On off period
		float chi = 0.95;

		NodeContainer nodes;
		nodes.Create (6); //index 0 is UE 0, index 1 is eNB 0, index 2 is server, index 3 is UE 1 index 4 is UE 2, index 5 is eNB 1

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

		PointToPointHelper p2pserver1; //Between eNB 1 and server
		p2pserver1.SetDeviceAttribute ("DataRate", StringValue ("1Gbps"));
		p2pserver1.SetChannelAttribute ("Delay", StringValue ("2ms"));
		p2pserver1.SetQueue ("ns3::DropTailQueue", "MaxSize", StringValue ("80000000p"));

		PointToPointHelper p2ue3; //Between UE 2 and eNB 1
		p2ue3.SetDeviceAttribute ("DataRate", StringValue ("1Gbps"));
		p2ue3.SetChannelAttribute ("Delay", StringValue ("1ms"));
		p2ue3.SetQueue ("ns3::DropTailQueue", "MaxSize", StringValue ("80000000p"));


		NetDeviceContainer devicesUeEnb;
		devicesUeEnb = pointToPoint.Install (nodes.Get(0), nodes.Get(1));

		NetDeviceContainer devicesEnbServer;
		devicesEnbServer = p2pserver.Install(nodes.Get(1), nodes.Get(2));

		NetDeviceContainer devicesUeEnb2;
		devicesUeEnb2 = p2pue2.Install(nodes.Get(3), nodes.Get(1));

		NetDeviceContainer devicesEnbServer2;
		devicesEnbServer2 = p2pserver1.Install(nodes.Get(5), nodes.Get(2));

		NetDeviceContainer devicesUeEnb3;
		devicesUeEnb3 = p2ue3.Install(nodes.Get(4), nodes.Get(5));

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
		devicesEnbServer2.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue (em4));
		devicesEnbServer2.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue (em4));

		Ptr<RateErrorModel> em5 = CreateObject<RateErrorModel>();
		em5->SetAttribute("ErrorRate", DoubleValue(errorRate5));
		devicesUeEnb3.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue (em5));
		devicesUeEnb3.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue (em5));

		 Ipv4AddressHelper ipv4;
		 ipv4.SetBase ("10.1.1.0", "255.255.255.0");
		 Ipv4InterfaceContainer interfacesUeEnb = ipv4.Assign (devicesUeEnb);

		 ipv4.SetBase("10.1.2.0", "255.255.255.0");
		 Ipv4InterfaceContainer interfacesUeEnb2 = ipv4.Assign (devicesUeEnb2);

		 ipv4.SetBase("10.1.3.0", "255.255.255.0");
		 Ipv4InterfaceContainer interfacesEnbServer = ipv4.Assign(devicesEnbServer);

		 ipv4.SetBase("10.1.4.0", "255.255.255.0");
		 Ipv4InterfaceContainer interfacesEnbServer2 = ipv4.Assign(devicesEnbServer2);

		 ipv4.SetBase("10.1.5.0", "255.255.255.0");
		 Ipv4InterfaceContainer interfacesUeEnb3 = ipv4.Assign(devicesUeEnb3);

		 //App for eNB0
		Ptr<EnbApp> app = CreateObject<EnbApp>();
		std::pair <Address, uint8_t> ueenbpair = std::make_pair(InetSocketAddress(interfacesUeEnb.GetAddress (0),dlPort),0);
		std::pair <Address, uint8_t> ueenbpair2 = std::make_pair(InetSocketAddress(interfacesUeEnb2.GetAddress (0),dlPort+1),1);
		std::vector<std::pair <Address, uint8_t>> pairHolder;
		pairHolder.push_back(ueenbpair);
		pairHolder.push_back(ueenbpair2);

		app->Setup(pairHolder, ulPort, 80, packetSize, eta, queueDelay, queueSize, dropFullProbability, InetSocketAddress(interfacesEnbServer.GetAddress(1), serverPortDl), serverPortUl, isExp4, period*decisionTime, chi);
		app->SetStartTime (startTime);
		app->SetStopTime (simTime);
		nodes.Get(1)->AddApplication(app);

		//App for UE0
		Ptr<UEApp> app1 = CreateObject<UEApp>();
		app1->Setup(InetSocketAddress(interfacesUeEnb.GetAddress(1), ulPort), InetSocketAddress(interfacesUeEnb.GetAddress(0), dlPort), packetSize, 0,  dataRate, ulPort, dlPort, decisionTime, posReward,
				negReward, alphaW, hiddenSize, alphaTheta, gamma, tcp1, false, 0, 0);
		app1->SetStartTime (startTime);
		app1->SetStopTime (simTime);
		nodes.Get(0)->AddApplication(app1);

		//App for UE1
		Ptr<UEApp> app2 = CreateObject<UEApp>();
		app2->Setup(InetSocketAddress(interfacesUeEnb2.GetAddress(1), ulPort+1), InetSocketAddress(interfacesUeEnb2.GetAddress(0), dlPort), packetSize, 1,  dataRate2, ulPort, dlPort+1, decisionTime, posReward,
				negReward, alphaW, hiddenSize, alphaTheta, gamma, tcp2, isExp4, period, 1);
		app2->SetStartTime(startTime);
		app2->SetStopTime(simTime);
		nodes.Get(3)->AddApplication(app2);

		//App for eNB1
		Ptr<EnbApp> app3 = CreateObject<EnbApp>();
		std::pair <Address, uint8_t> ueenbpair3 = std::make_pair(InetSocketAddress(interfacesUeEnb3.GetAddress (0),dlPort1),0);
		std::vector<std::pair <Address, uint8_t>> pairHolder1;
		pairHolder1.push_back(ueenbpair3);
		app3->Setup(pairHolder1, ulPort1, 80, packetSize, eta, queueDelay1, queueSize1, dropFullProbability, InetSocketAddress(interfacesEnbServer2.GetAddress(1), serverPortDl + 1), serverPortUl + 1, isExp4, period*decisionTime, chi);
		nodes.Get(5)->AddApplication(app3);
		//App for UE2
		Ptr<UEApp> app4 = CreateObject<UEApp>();
		app4->Setup(InetSocketAddress(interfacesUeEnb3.GetAddress(1), ulPort1), InetSocketAddress(interfacesUeEnb3.GetAddress(0), dlPort1), packetSize, 0,  dataRate3, ulPort1, dlPort1, decisionTime, posReward,
						negReward, alphaW, hiddenSize, alphaTheta, gamma, tcp3, false, 0, 2);
		app4->SetStartTime(startTime);
		app4->SetStopTime(simTime);
		nodes.Get(4)->AddApplication(app4);

		Ptr<ServerApp> serverApp = CreateObject<ServerApp>();
		serverApp->Setup(InetSocketAddress(interfacesEnbServer.GetAddress(0), serverPortUl), InetSocketAddress(interfacesEnbServer2.GetAddress(0), serverPortUl + 1), serverPortDl);
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
		 plot.AppendExtra ("set yrange [0:+600]");
		 Gnuplot2dDataset dataset;
		 Gnuplot2dDataset ds;
		 dataset.SetTitle("UE 0");
		 dataset.SetStyle (Gnuplot2dDataset::LINES);
		 ds.SetTitle("UE 1");
		 ds.SetStyle (Gnuplot2dDataset::LINES);
		 Time counter_time = decisionTime;
		 uint32_t count_cwnd = 0;
		 uint32_t count_cwnd4 = 0;
		 double cwnd1_avg = 0;
		 double cwnd2_avg = 0;
		 double cwnd1_avgOn = 0;
		 double cwnd2_avgOn = 0;
		 bool isOn = true;
		 uint32_t counterState = 0;
		 for(uint32_t i = 0; i < std::min(trackCWND.size(), trackCWND2.size()); ++i){
			 count_cwnd++;
			 uint64_t t = counter_time.GetMilliSeconds();
			 dataset.Add(t, trackCWND[i]);
			 ds.Add(counter_time.GetMilliSeconds(), trackCWND2[i]);
			 cwnd1_avg += (trackCWND[i] - cwnd1_avg)/count_cwnd;
			 cwnd2_avg += (trackCWND2[i] - cwnd2_avg)/count_cwnd;
			 if(counterState > period){
				 counterState = 0;
				 isOn = !isOn;
			 }
			 if(isExp4 && isOn){
				 count_cwnd4++;
				 cwnd1_avgOn += (trackCWND[i] - cwnd1_avgOn)/count_cwnd4;
				 cwnd2_avgOn += (trackCWND2[i] - cwnd2_avgOn)/count_cwnd4;
			 }
			 counter_time += decisionTime;
			 counterState++;
		 }
		 plot.AddDataset(dataset);
		 plot.AddDataset(ds);
		 std::ofstream plotFile (plotFileName.c_str());
		 plot.GenerateOutput (plotFile);
		 plotFile.close ();

		 fileNameWithNoExtension = "throughput-plot";
		 graphicsFileName  = fileNameWithNoExtension + ".png";
		 plotFileName  = fileNameWithNoExtension + ".plt";
		 Gnuplot plot2 (graphicsFileName);
		 plot2.SetTitle ("Throughput Validity");
		 plot2.SetTerminal ("png");
		 plot2.SetLegend ("time (ms)", "Throughput (Mbps)");
		 plot2.AppendExtra ("set yrange [0:+4]");
		 //plot.AppendExtra ("set yrange [0:+1000]");
		 Gnuplot2dDataset dataset2;
		 Gnuplot2dDataset dataset3;
		 dataset2.SetTitle("Estimated Throughput");
		 dataset3.SetTitle("Expected Throughput");
		 dataset2.SetStyle (Gnuplot2dDataset::LINES);
		 dataset3.SetStyle (Gnuplot2dDataset::LINES);
		 Time movingTime = trackCalculatedThroughput[0].first;
		 Time basedTime = trackCalculatedThroughput[0].first;

		 double moving_average = 0;
		 uint32_t count = 0;
		 for(uint32_t i = 0; i < trackCalculatedThroughput.size(); ++i){
			 movingTime = trackCalculatedThroughput[i].first;
			 if(movingTime - basedTime < 5*decisionTime){
				 count++;
				 moving_average += (trackCalculatedThroughput[i].second - moving_average)/count;
			 }
			 else{
				 dataset2.Add(trackCalculatedThroughput[i].first.GetMilliSeconds(), moving_average);
				 dataset3.Add(trackCalculatedThroughput[i].first.GetMilliSeconds(),trackExpectedThroughput);
				 moving_average = 0;
				 count = 0;
				 basedTime = movingTime;
			 }
		 }

		 plot2.AddDataset(dataset2);
		 plot2.AddDataset(dataset3);
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
		 }
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
			 std::cout<<"cwnd avg of UE 1 when on = " << (cwnd2_avgOn*dataRate.GetBitRate()/1e6)  << " Mbps"<<std::endl;
		 }
		 else{
			 std::cout<<"Expected throughput = "<< trackExpectedThroughput <<" Mbps" << std::endl;
			 std::cout<<"Average throughput of UE 0 = "<< (cwnd1_avg*dataRate.GetBitRate()/1e6) << " Mbps" <<std::endl;
			 std::cout<<"Average throughput of UE 1 = "<< (cwnd2_avg*dataRate2.GetBitRate()/1e6) << " Mbps" <<std::endl;
		 }

		 return 0;
}





