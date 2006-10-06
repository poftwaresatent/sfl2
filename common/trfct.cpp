#include <iostream>

using namespace std;

class RobotClient;

class RobotServer {
public:
  RobotServer(RobotClient * client);
  void CallHook();
  int GetId();
private:
  static int m_next_id;
  int m_id;
  RobotClient * m_client;
};

class RobotClient {
public:
  RobotClient();
  virtual ~RobotClient();
  virtual void Hook() = 0;
  int GetId();
  RobotServer * GetServer();
private:
  RobotServer m_server;
};

class Robox: public RobotClient {
public:
  virtual void Hook() { cout << "hello from robox, id " << GetId() << "\n"; }
};

int main(int argc, char ** argv)
{
  Robox robox;
  RobotServer * robox_server(robox.GetServer());
  cout << "calling robox:\n  ";
  robox.Hook();
  cout << "calling server:\n  ";
  robox_server->CallHook();
}

int RobotServer::m_next_id(42);
RobotServer::RobotServer(RobotClient * client)
  : m_id(m_next_id++), m_client(client) {}
void RobotServer::CallHook() { m_client->Hook(); }
int RobotServer::GetId() { return m_id; }

RobotClient::RobotClient(): m_server(this) {}
RobotClient::~RobotClient() {}
int RobotClient::GetId() { return m_server.GetId(); }
RobotServer * RobotClient::GetServer() { return & m_server; }
