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
bool readThroughput;
bool readQueue;
bool readReward;
bool readCWND;
bool readPolicy;
bool readBias;
std::vector<uint32_t> trackCWND;
double trackExpectedThroughput;
std::vector<std::pair<Time, double>> trackCalculatedThroughput;
//------------------------------ Global variable definition ends here --------------------------------

//------------------------------ Global function definition ends here --------------------------------

//------------------------------ Global function definition ends here --------------------------------

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
										//TODO: Add 2 more bits: ECE flag and CWR flag

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
			float m_eta, Time timeDelay, uint16_t maxQueue, float dropP);

protected:
	void handleRead(Ptr<Socket> socket);
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
	//TODO: Add sendPacketToUE
	void setExpectedThroughput(bool isComingUE);

	std::vector<Ptr<Socket>>     m_socketReceivers;
	std::vector<Ptr<Socket>>  m_socketSenders;
	std::vector<std::vector<Time>> m_lastReceivedTimes;
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
    float m_expectedThroughput; //Should be the same across the user to ensure fairness
    float accumulatedBytes; //TODO to support N users
    double current_throughput;
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
		float eta, Time timeDelay, uint16_t maxQueue, float dropP)
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
}

void
EnbApp::setExpectedThroughput(bool isComingUE){


	m_expectedThroughput = 1.1*m_packetSize*8/(m_serviceTime.GetMicroSeconds()) ; //The coefficient 1.1 vary with maximum queue size. The unit is in Mbps
	//m_expectedThroughput = 2.7;

	if(isComingUE){
		m_numberOfUE++;
		m_expectedThroughput = m_expectedThroughput /m_numberOfUE;
	}
	else{
		m_numberOfUE--;
		if(m_numberOfUE != 0){
			m_expectedThroughput = m_expectedThroughput/m_numberOfUE;
		}
	}
	trackExpectedThroughput = m_expectedThroughput;
}
void
EnbApp::StartApplication (void)
{
  std::cout<< " Starting ENB Application" << std::endl;
  current_throughput = 0;
  m_running = true;
  m_numberOfUE = 0;
  m_queueCounter = 0;
  Time current_time = Simulator::Now();
  m_lastReceivedTimes.resize(m_addressNodePairs.size());
  for(uint8_t i = 0; i < m_addressNodePairs.size(); ++i){
	  for(uint8_t j = 0; j < 3 ; ++j){
		  m_lastReceivedTimes[i].push_back(current_time);
	  }

  }
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
  }

    sample = CreateObject<UniformRandomVariable>();
	sample->SetAttribute("Min", DoubleValue(0));
	sample->SetAttribute("Max", DoubleValue(1));
    setExpectedThroughput(true);

}

void
EnbApp::handleRead(Ptr<Socket> socket){
	++m_queueCounter;
	if(readQueue){
		if(m_queueCounter > 30)
		std::cout<<"Current scheduler queue size: "<<m_queueCounter<<std::endl;
	}
	if(!m_running) {return;}
	Ptr<Packet> packet;
	Address sender;
	while((packet = socket->RecvFrom(sender))){
		CustomECNTag ecnTag;
		CustomTCPTag  tcpTag;
		TimestampTag timeTag;
		bool found_ecn = packet->PeekPacketTag(ecnTag);
		bool found_tcp = packet->PeekPacketTag(tcpTag);
		bool found_time = packet->PeekPacketTag(timeTag);
		if(!found_ecn){
			std::cout<<"Fatal Error: the ECN tag is not detected!"<<std::endl;
			exit(-1);
		}
		if(!found_tcp){
			std::cout<<"Fatal Error: Cannot found the tcp tag!"<<std::endl;
			exit(-1);
		}
		if(!found_time){
			std::cout<<"Fatal Error: Timestamp not found!"<<std::endl;
			exit(-1);
		}
		accumulatedBytes += packet->GetSize();
		uint8_t ueId = tcpTag.getId();
		if(m_queueCounter > m_maxQueueSize){
			 double val = sample->GetValue();
			 if(val < m_dropProbability) {
				 --m_queueCounter;
				 return;}
			ecnTag.setBits(3); //Congested State
			Simulator::Schedule (m_queueCounter*m_serviceTime, &EnbApp::SendPacketToUE, this, ueId, ecnTag, timeTag);
			return;
		}

		Time current_time = Simulator::Now();

		//float current_throughput = 3*packet->GetSize()*8.0/(current_time.GetSeconds() + m_lastReceivedTimes[ueId][0].GetSeconds() + m_lastReceivedTimes[ueId][1].GetSeconds() -  3*m_lastReceivedTimes[ueId][2].GetSeconds())/1e6;
		if(current_time - m_lastReceivedTimes[ueId][0] > m_serviceTime){
			current_throughput = 0.7*current_throughput + 0.3*accumulatedBytes*8/(current_time - m_lastReceivedTimes[ueId][0]).GetMicroSeconds();
			accumulatedBytes = 0;
			m_lastReceivedTimes[ueId][0] = current_time;
		}
		trackCalculatedThroughput.push_back(std::make_pair(current_time, current_throughput));
		if(readThroughput){
			std::cout<<"Current throughput: "<<current_throughput << std::endl;
			std::cout<<"Compared throughput:"<<m_expectedThroughput<<std::endl;
		}
		/*m_lastReceivedTimes[ueId][2] = m_lastReceivedTimes[ueId][1];
		m_lastReceivedTimes[ueId][1] = m_lastReceivedTimes[ueId][0];
		m_lastReceivedTimes[ueId][0] = current_time;*/
		if(ecnTag.isPositiveReward()){
			if(current_throughput > m_expectedThroughput|| current_throughput < (m_eta*m_expectedThroughput)){
				//Send to UE directly
				ecnTag.setBits(1); //You are penalized!
				Simulator::Schedule (m_queueCounter*m_serviceTime, &EnbApp::SendPacketToUE, this, ueId, ecnTag, timeTag);
				return;
			}
		}

		Simulator::Schedule (m_queueCounter*m_serviceTime, &EnbApp::SendPacketToUE, this, ueId, ecnTag, timeTag);


	}

}

void
EnbApp::SendPacketToUE(uint8_t nodeId, CustomECNTag ecnTag, TimestampTag timeTag){
	//TODO: dont loss the id for next
	if(!m_running) {return;}
	--m_queueCounter;
	Ptr<Packet> packet = Create<Packet> (m_packetSize);
	packet->AddPacketTag(ecnTag);
	packet->AddPacketTag(timeTag);
	Ptr<Socket> sendSocket = m_socketSenders[nodeId];

	sendSocket->Send(packet);
	//ScheduleTx ();
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
		double ratioRTT;
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
	sample->SetAttribute("Variance", DoubleValue(2.0/static_cast<double>(m_hiddenSize)));
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
	sample->SetAttribute("Variance", DoubleValue(1));
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
	RLCompute(int action_bound, double alphaW, uint16_t hiddenSize, double alphaTheta, double gamma);
	int compute(float reward, State currState);

private:
	int actionSelect(State state);
	void updateTheta();
	double getActionValue(State state, int action);
	void updateStochasticPolicy(State state);
	std::vector<double> getFeatureVector(State s, int a);

	std::vector<double> m_thetaRTT;
	std::vector<double> m_thetaACK;
	std::vector<double> m_policy;
	int m_actionBound;
	float m_alphaTheta;
	float m_gamma;
	State m_lastState;
	int m_lastAction;
	bool m_firstTime;
	bool m_isTerminal;
	NN* m_nn;
};
RLCompute::RLCompute(int action_bound, double alphaW, uint16_t hiddenSize, double alphaTheta, double gamma){
	m_actionBound = action_bound;
	Ptr<NormalRandomVariable> sampleX = CreateObject<NormalRandomVariable>();
	sampleX->SetAttribute("Mean", DoubleValue(0));
	sampleX->SetAttribute("Variance", DoubleValue(1));
	for(int i =0; i < 2*m_actionBound + 1; ++i){
		m_thetaACK.push_back(sampleX->GetValue());
		m_thetaRTT.push_back(sampleX->GetValue());
	}
    m_nn = new NN(alphaW, hiddenSize);
	m_alphaTheta = alphaTheta;
	m_gamma = gamma;
	m_firstTime = true;
	m_isTerminal = false;
	m_lastAction = 0;
}
int
RLCompute::compute(float reward, State currState){
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
		double td_error = reward  - getActionValue(m_lastState, m_lastAction);
		m_nn->updateWeights(td_error, getFeatureVector(m_lastState, m_lastAction));
		m_isTerminal = true;
		return 0;
	}
	updateTheta();
	updateStochasticPolicy(currState);
	int next_action =  actionSelect(m_lastState);
	double td_error = reward + m_gamma*getActionValue(currState, next_action) - getActionValue(m_lastState, m_lastAction);
	m_nn->updateWeights(td_error, getFeatureVector(m_lastState, m_lastAction));
	if(readPolicy){
		std::cout<<"[ ";
		for(int i = 0; i < 2*m_actionBound ;++i){
			std::cout<<m_policy[i]<<" , ";
		}
		std::cout<<m_policy[2*m_actionBound]<<" ]"<<std::endl;
	}
	m_lastState = currState;
	m_lastAction = next_action;
	return next_action;
}

void
RLCompute::updateTheta(){
	double Q = getActionValue(m_lastState,m_lastAction);
	double sum_RTT = 0;
	double sum_ACK = 0;
	for(int i = 0; i < 2*m_actionBound + 1; ++i){
		sum_ACK += m_policy[i]*m_thetaACK[i]*m_lastState.ACKRatio;
		sum_RTT += m_policy[i]*m_thetaRTT[i]*m_lastState.ratioRTT;
	}
	for(int i = 0; i < 2*m_actionBound + 1; ++i){
		m_thetaACK[i] += m_alphaTheta*Q*(m_thetaACK[i]*m_lastState.ACKRatio - sum_ACK);
		m_thetaRTT[i] += m_alphaTheta*Q*(m_thetaRTT[i]*m_lastState.ratioRTT - sum_RTT);
	}
}


void
RLCompute::updateStochasticPolicy(State state){
	 std::vector<double> h;
	 double maxVal = -10000000;
	 for(int i = 0; i < 2*m_actionBound + 1; ++i){
		 double temp = m_thetaACK[i]*state.ACKRatio + m_thetaRTT[i]*state.ratioRTT;
		 if(temp > maxVal) maxVal = temp;
		 h.push_back(temp);
	 }
	 double normalization = 0;
	 for(int i = 0; i < 2*m_actionBound + 1; ++i){
		 h[i] -= maxVal;
		 h[i] = std::exp(h[i]);
		 normalization += h[i];
	 }
	 for(int i = 0; i < 2*m_actionBound + 1; ++i){
		 h[i] = h[i]/normalization;
	 }
	m_policy = h;

}

std::vector<double>
RLCompute::getFeatureVector(State s, int a){
	std::vector<double> inp;
	inp.push_back(s.ACKRatio);
	inp.push_back(s.ratioRTT);
	inp.push_back(a);
	return inp;
}

double
RLCompute::getActionValue(State state, int action){
	return m_nn->getValue(getFeatureVector(state,action));
}
int
RLCompute::actionSelect(State state){
	Ptr<UniformRandomVariable> sample = CreateObject<UniformRandomVariable>();
		sample->SetAttribute("Min", DoubleValue(0));
		sample->SetAttribute("Max", DoubleValue(1));
		double rand_number = sample->GetValue();
		double sum = 0;
		for(int i = 0; i < 2*m_actionBound+1; ++i){
			sum += m_policy[i];
			if(sum > rand_number) return (i - m_actionBound);
		}
		return 0;

}

//------------------------------RLCompute class definition ends here -----------------------------

//------------------------------UEApp class definition starts here ------------------------------------

class UEApp : public Application{
public:
	UEApp();
	virtual ~UEApp();

	void Setup (Address send, Address receive,  uint32_t packetSize, uint8_t ueId, DataRate dataRate,  uint16_t ulPort, uint16_t dlPort, Time decisionPeriod,
			float positiveReward, float negativeReward, double alphaW, uint16_t hiddenSize, double alphaTheta, double gamma, int actionBound);

protected:
	void handleRead(Ptr<Socket> socket); //For congestion control
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
	uint32_t m_cwndCounter;
	uint32_t m_RxcwndCounter;
	bool m_hasCalledNOAck;
	Time m_decisionPeriod;
	float m_positiveReward;
	float m_negativeReward;
	RLCompute* rl;
	double m_alphaW;
	double m_alphaTheta;
	double m_gamma;
	uint16_t m_hiddenSize;
	bool m_isCongested;
	int m_actionBound;

};

UEApp:: UEApp () {}

UEApp:: ~UEApp() {
	m_socketReceiver = 0;
	m_socketSender = 0;
}

void
UEApp::Setup(Address addressSend, Address addressReceive, uint32_t packetSize, uint8_t ueId,  DataRate dataRate,  uint16_t ulPort, uint16_t dlPort, Time decisionPeriod,
		float positiveReward, float negativeReward, double alphaW, uint16_t hiddenSize, double alphaTheta, double gamma, int actionBound){
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
	m_actionBound = actionBound;
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
  rl = new RLCompute(m_actionBound, m_alphaW, m_hiddenSize, m_alphaTheta, m_gamma);
  m_RxcwndCounter = 0;
  m_cwndCounter = m_cwnd;
  m_averageReward = 0;
  m_minRTT = 1e10; // set to arbitrary large value
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
	if(m_cwndCounter == 0 && !m_hasCalledNOAck) {
		ScheduleTxNoACK();
		return;
	}
	if(m_cwndCounter == 0){
		ScheduleTx();
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
	//std::cout<<"Sending because of timeout" << std::endl;
	m_cwndCounter = m_cwnd;
	 m_RxcwndCounter = 0;
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
			else if(tag.isCongested()){
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
		 // std::cout<<"estimated RTT = " << m_estimatedRTT<< " ms" << std::endl;
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
	if(m_cwndCounter == 0){
		//Agent must do nothing when timeout
		Simulator::Schedule (m_decisionPeriod, &UEApp::updateReward, this);
		return;
	}
	State curr_state;
	if(m_countTx != 0){
		curr_state.ACKRatio = static_cast<double>(m_countRx)/m_countTx ;
	}
	else{
		curr_state.ACKRatio = 0;
	}
	if(m_minRTT != 0){
		curr_state.ratioRTT = m_estimatedRTT/m_minRTT;
	}
	else{
		curr_state.ratioRTT = 1;
	}
	if(readReward){
		std::cout<<"Get average reward = " << m_averageReward << std::endl;
		if(m_countTx != 0){
		std::cout<<"ACK ratio =  " << curr_state.ACKRatio << std::endl;
		}
		else{
			std::cout<<"count TX is zero"<<std::endl;
		}
		if(m_minRTT != 0)
		std::cout<<"RTT ratio = " << curr_state.ratioRTT << std::endl;
		else
			std::cout<<"m_minRTT is zero"<<std::endl;
	}

	if(m_isCongested){
		curr_state.isTerminal = true;
		 rl->compute(m_averageReward, curr_state);
		if(m_cwnd > 10){
			m_cwnd = m_cwnd *3/4;
		}
		else{
			m_cwnd = 10;
		}
	}
	else{
		curr_state.isTerminal = false;
		int change = rl->compute(m_averageReward, curr_state);
		int test_cwnd = m_cwnd + change;
		if(test_cwnd < 1){
			//NO change
		}
		else{
			m_cwnd += change;
		}
	}
	if(m_cwnd < 1){
				m_cwnd = 1;
		}
	if(readCWND){
		std::cout<<"Current CWND = " << m_cwnd << std::endl;
	}
	trackCWND.push_back(m_cwnd);
	m_countRx= 0;
	m_countTx = 0;
	m_averageReward = 0;
	Simulator::Schedule (m_decisionPeriod, &UEApp::updateReward, this);
}

int
main (int argc, char *argv[])
{

	readThroughput = false;
	readQueue = false;
	readReward =false;
	readCWND = false;
	readPolicy = false;
	readBias = false;

	 uint16_t ulPort = 10000;  //Uplink port offset
	 uint16_t dlPort = 20000;  //Downlink port offset
	 Time startTime = Seconds(0.03);  //start application time
	 Time simTime = Seconds(180); //simulation time
	 uint16_t packetSize = 100; //packet size of UDP in bits
	 double errorRate = 0.0001; //Dropping rate due to receiving side of eNB
	 DataRate dataRate = DataRate("0.01Mbps");
	 Time queueDelay = MicroSeconds(100);
	 float eta = 0.9;
	 uint16_t queueSize= 50;
	 double dropFullProbability = 0.9;
	 float posReward = 20;
	 float negReward = -20;
	 Time decisionTime = MilliSeconds(20);
	 double alphaW = 0.0001;
	 uint32_t hiddenSize = 7;
	 double alphaTheta = 0.00001;
	 double gamma = 0.9;
	 int actionBound = 5;


	 NodeContainer nodes;
	 nodes.Create (2);

	  PointToPointHelper pointToPoint;
	  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("1Gbps"));
	  pointToPoint.SetChannelAttribute ("Delay", StringValue ("10ms"));
	  pointToPoint.SetQueue ("ns3::DropTailQueue", "MaxSize", StringValue ("80000000p"));

	  NetDeviceContainer devices;
		devices = pointToPoint.Install (nodes);

		InternetStackHelper stack;
		stack.Install (nodes);
		Ptr<RateErrorModel> em = CreateObject<RateErrorModel> ();
		em->SetAttribute ("ErrorRate", DoubleValue (errorRate));
		devices.Get (1)->SetAttribute ("ReceiveErrorModel", PointerValue (em));
		//devices.Get (0)->SetAttribute ("ReceiveErrorModel", PointerValue (em));

		 Ipv4AddressHelper address;
		 address.SetBase ("10.1.1.0", "255.255.255.0");
		 Ipv4InterfaceContainer interfaces = address.Assign (devices);



		 //Remember node 0 is UE and node 1 is eNB
		Ptr<EnbApp> app = CreateObject<EnbApp>();
		std::pair <Address, uint8_t> ueenbpair = std::make_pair(InetSocketAddress(interfaces.GetAddress (0),dlPort),0);
		std::vector<std::pair <Address, uint8_t>> pairHolder;
		pairHolder.push_back(ueenbpair);

		app->Setup(pairHolder, ulPort, 80, packetSize, eta, queueDelay, queueSize, dropFullProbability); //We dont care about SGW for now
		nodes.Get(1)->AddApplication(app);
		app->SetStartTime (startTime);
		app->SetStopTime (simTime);

		Ptr<UEApp> app1 = CreateObject<UEApp>();
		app1->Setup(InetSocketAddress(interfaces.GetAddress(1), ulPort), InetSocketAddress(interfaces.GetAddress(0), dlPort), packetSize, 0,  dataRate, ulPort, dlPort, decisionTime, posReward,
				negReward, alphaW, hiddenSize, alphaTheta, gamma, actionBound);
		nodes.Get(0)->AddApplication(app1);
		app1->SetStartTime (startTime);
		app1->SetStopTime (simTime);

		Simulator::Stop (simTime);
		Simulator::Run ();
		Simulator::Destroy ();

		 std::string fileNameWithNoExtension = "cwnd-plot";
		 std::string graphicsFileName  = fileNameWithNoExtension + ".png";
		 std::string plotFileName  = fileNameWithNoExtension + ".plt";
		 std::string dataTitle = "CWND";

		 Gnuplot plot (graphicsFileName);
		 plot.SetTitle ("CWND-Softmax");
		 plot.SetTerminal ("png");
		 plot.SetLegend ("time (ms)", "CWND");
		 plot.AppendExtra ("set yrange [0:+1000]");
		 Gnuplot2dDataset dataset;
		 dataset.SetTitle(dataTitle);
		 dataset.SetStyle (Gnuplot2dDataset::LINES);
		 Time counter_time = decisionTime;
		 for(uint32_t i = 0; i < trackCWND.size(); ++i){
			 dataset.Add(counter_time.GetMilliSeconds(), trackCWND[i]);
			 counter_time += decisionTime;
		 }
		 plot.AddDataset(dataset);
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
			 if(movingTime - basedTime < MilliSeconds(20)){
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
		 return 0;
}





