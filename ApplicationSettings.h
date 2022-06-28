#ifndef APPLICATIONSETTINGS_H
#define APPLICATIONSETTINGS_H

#include <QObject>
#include <vector>

#include "common/YamlConfig.h"

class ApplicationSettings : public QObject {
  Q_OBJECT

 public:
  ApplicationSettings();

  int load();
  void save(std::string server_address, std::string creds_type);
  std::pair<std::string, std::string> getSettings();
  std::vector<std::string> getConfigsList();

  std::string serverAddress() { return server_ip_and_port; };
  std::string serverIp() { return server_ip; };
  std::string serverCredentials() { return creds_type; };
  std::string serverAddrAndCreds() { return addr_and_creds; };

  const config::PolicyData& getPolicyData();
  const config::CallPolicyData& getCallPolicyData();
  const config::CallUserPolicyData& getCallUserPolicyData();
  const config::SecSettingsData& getSecSettingsData();
  const config::PolicyTemplatesData& getPolicyTemplatesData();
  void setPolicyTemplatesData(const config::PolicyTemplatesData& data);

 private:
  std::string m_path;
  YamlConfig m_config;
  YamlConfig m_templates;
  std::string server_ip_and_port;
  std::string server_ip;
  std::string creds_type;
  std::string addr_and_creds;

  config::PolicyData policyData{};
  config::CallPolicyData callPolicyData{};
  config::CallUserPolicyData callUserPolicyData{};
  config::SecSettingsData secSettingsData{};
  config::PolicyTemplatesData policyTemplatesData{};

  const std::string APP_SETTINGS = "/AppSettings.txt";
  const std::string CONFIG_PATH = "/config.yaml";
  const std::string POLICIES_PATH = "/policies.yaml";
};

#endif  // APPLICATIONSETTINGS_H
