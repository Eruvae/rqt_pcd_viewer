#ifndef RQT_PCD_VIEWER_H
#define RQT_PCD_VIEWER_H

#include <QWidget>
#include <QFileSystemModel>
#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <QModelIndex>
#include <QString>
#include <QStringList>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rqt_gui_cpp/plugin.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>

#include "ui_rqt_pcd_viewer.h"

namespace rqt_pcd_viewer
{

struct InstanceSettings
{
  QString lastFolder;
};

class RqtPcdViewer;

class RqtPcdViewerWidget : public QWidget
{
public:
  explicit RqtPcdViewerWidget(RqtPcdViewer* plugin, QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());

protected:
  virtual void resizeEvent(QResizeEvent *event);

private:
  RqtPcdViewer *plugin;
};

class RqtPcdViewer : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

  friend class RqtPcdViewerWidget;

public:
  RqtPcdViewer();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
  virtual bool hasConfiguration() const;
  virtual void triggerConfiguration();

private slots:
  void on_selectFolderButton_clicked();
  void on_fileTreeView_doubleClicked(const QModelIndex &index);
  void on_nextPcdButton_clicked();
  void on_previousPcdButton_clicked();
  void on_publishingTimer_timeout();
  void on_topicLineEdit_textChanged(const QString &topic);
  void on_visualizeCheckBox_toggled(bool checked);
  void startPublishing();
  void stopPublishing();

private:
  QTimer publishingTimer;

  pcl::PCLPointCloud2::Ptr cloud;
  ros::Publisher pointcloud_pub;

  bool viewer_initialized;
  void initializeViewer();
  void deinitializeViewer();

  bool loadPcd(const QModelIndex &index);
  void visualizePointcloud();
  void publishPointcloud();
  void setSelectedPcd(QModelIndex index);
  void clearSelectedPcd();
  void pluginSettingsToUi();
  void uiToPluginSettings();
  void applySettings();

  // UI members
  Ui::RqtPcdViewerWidgetUi ui;
  RqtPcdViewerWidget* widget;

  pcl::visualization::PCLVisualizer::Ptr viewer;

  QFileSystemModel* folder_model;

  // Currently loaded pcds
  bool pcd_loaded;
  QModelIndex selected_pcd;

  int viewport;

  // Settings
  InstanceSettings settings;
};

}

#endif // RQT_PCD_VIEWER_H
