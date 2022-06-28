#include "ApplicationSettings.h"

#include <qlibrary.h>

#include <QApplication>
#include <QFile>
#include <QFileInfo>
#include <QSettings>

#include "Loggers.h"
#include "Settings.h"

ApplicationSettings::ApplicationSettings()
    : m_path(config::FILE_CONFIG_PATH), m_config() {
  if (m_path.empty()) {
    m_path = QApplication::applicationDirPath().toStdString();
  }
  info(m_path);
};

std::pair<std::string, std::string> ApplicationSettings::getSettings() {
  return {server_ip_and_port, creds_type};
}

int ApplicationSettings::load() {
  auto app_settings_filename = QString::fromStdString(m_path + APP_SETTINGS);
  if (QFileInfo::exists(app_settings_filename)) {
    QSettings app_settings(app_settings_filename, QSettings::IniFormat);
    app_settings.beginGroup("AppSettings");
    server_ip_and_port =
        app_settings.value("server_addr").toString().toStdString();
    creds_type =
        app_settings.value("credentials_type").toString().toStdString();
    app_settings.endGroup();

    addr_and_creds = creds_type + "@" + server_ip_and_port;
    server_ip = server_ip_and_port.substr(0, server_ip_and_port.find(":"));
  }

  m_config.load(m_path + CONFIG_PATH);
  m_templates.load(m_path + POLICIES_PATH);
  return 0;
}

void ApplicationSettings::save(std::string server_address,
                               std::string creds_type) {
  QString app_settings_filename = QString::fromStdString(m_path + APP_SETTINGS);
  QSettings app_settings(app_settings_filename, QSettings::IniFormat);
  app_settings.beginGroup("AppSettings");
  app_settings.setValue("server_addr", QString::fromStdString(server_address));
  app_settings.setValue("credentials_type", QString::fromStdString(creds_type));
  app_settings.endGroup();
}

std::vector<std::string> ApplicationSettings::getConfigsList() {
  std::vector<std::string> configs;
  configs.emplace_back(m_path + APP_SETTINGS);
  configs.emplace_back(m_path + CONFIG_PATH);
  configs.emplace_back(m_path + POLICIES_PATH);
  return configs;
}

const config::PolicyData& ApplicationSettings::getPolicyData() {
  if (policyData.empty()) {
    auto node = m_config.getNode("policy.user");
    policyData = node.as<config::PolicyData>();
  }

  return policyData;
}

const config::CallPolicyData& ApplicationSettings::getCallPolicyData() {
  if (callPolicyData.empty()) {
    auto node = m_config.getNode("policy.call");
    callPolicyData = node.as<config::CallPolicyData>();
  }

  return callPolicyData;
}

const config::CallUserPolicyData& ApplicationSettings::getCallUserPolicyData() {
  if (callUserPolicyData.empty()) {
    auto node = m_config.getNode("policy.user_call");
    callUserPolicyData = node.as<config::CallUserPolicyData>();
  }

  return callUserPolicyData;
}

const config::SecSettingsData& ApplicationSettings::getSecSettingsData() {
  if (secSettingsData.empty()) {
    auto node = m_config.getNode("sec_settings");
    secSettingsData = node.as<config::SecSettingsData>();
  }

  return secSettingsData;
}

const config::PolicyTemplatesData&
ApplicationSettings::getPolicyTemplatesData() {
  if (policyTemplatesData.empty()) {
    auto node = m_templates.getNode("policy");
    policyTemplatesData = node.as<config::PolicyTemplatesData>();
  }

  return policyTemplatesData;
}

void ApplicationSettings::setPolicyTemplatesData(
    const config::PolicyTemplatesData& data) {
  policyTemplatesData = data;
  m_templates.setNode("policy", data);
}
