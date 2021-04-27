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
#include <unordered_map>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>

#include "ui_rqt_pcd_viewer.h"

namespace rqt_pcd_viewer
{

static constexpr size_t NUM_VIEWS = 2;

struct InstanceSettings
{
  QStringList lastFolders;
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
  void on_selectFolderButton_clicked(size_t vpind);
  void on_fileTreeView_doubleClicked(const QModelIndex &index, size_t vpind);
  void on_nextPcdButton_clicked(size_t vpind);
  void on_previousPcdButton_clicked(size_t vpind);

private:
  bool loadPcd(const QModelIndex &index, size_t vpind);
  void setSelectedPcd(QModelIndex index, size_t vpind);
  void clearSelectedPcd(size_t vpind);
  void pluginSettingsToUi();
  void uiToPluginSettings();
  void applySettings();

  // UI members
  Ui::RqtPcdViewerWidgetUi ui;
  RqtPcdViewerWidget* widget;

  pcl::visualization::PCLVisualizer::Ptr viewer;

  QFileSystemModel* folder_model[NUM_VIEWS];

  // GUI elements
  QTreeView *fileTreeView[NUM_VIEWS];
  QPushButton *selectFolderButton[NUM_VIEWS];
  QToolButton *previousPcdButton[NUM_VIEWS];
  QLabel *curPcdLabel[NUM_VIEWS];
  QToolButton *nextPcdButton[NUM_VIEWS];

  // Currently loaded pcds
  bool pcd_loaded[NUM_VIEWS];
  QModelIndex selected_pcd[NUM_VIEWS];

  std::unordered_map<size_t, int> vp_map;

  // Settings
  InstanceSettings settings;
};

}

#endif // RQT_PCD_VIEWER_H
