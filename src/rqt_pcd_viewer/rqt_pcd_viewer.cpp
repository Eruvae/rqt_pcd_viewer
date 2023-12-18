#include "rqt_pcd_viewer/rqt_pcd_viewer.h"

#include <pluginlib/class_list_macros.h>
#include <QFileDialog>
#include <QInputDialog>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io_exception.h>
#include <unordered_set>

namespace rqt_pcd_viewer
{

// Widget class for resize event

RqtPcdViewerWidget::RqtPcdViewerWidget(RqtPcdViewer* plugin, QWidget* parent, Qt::WindowFlags f)
  : QWidget(parent, f), plugin(plugin)
{
}

void RqtPcdViewerWidget::resizeEvent(QResizeEvent *event)
{
}

// Main class

RqtPcdViewer::RqtPcdViewer() :
  rqt_gui_cpp::Plugin(),
  publishingTimer(this),
  cloud(nullptr),
  viewer_initialized(false),
  widget(Q_NULLPTR),
  viewer(Q_NULLPTR)
{
  folder_model = nullptr;
  pcd_loaded = false;
  selected_pcd = QModelIndex();
  
  setObjectName("RqtPcdViewer");
}

void RqtPcdViewer::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  // QStringList argv = context.argv();
  // create QWidget
  widget = new RqtPcdViewerWidget(this);
  // extend the widget with all attributes and children from UI file
  ui.setupUi(widget);
  // add widget to the user interface
  context.addWidget(widget);

  if (ui.visualizeCheckBox->isChecked())
  {
    ui.pcdView->setVisible(true);
    initializeViewer();
  }
  else
  {
    ui.pcdView->setVisible(false);
  }

  pointcloud_pub = getNodeHandle().advertise<sensor_msgs::PointCloud2>(ui.topicLineEdit->text().toStdString(), 1);

  connect(ui.selectFolderButton, &QAbstractButton::clicked, this, &RqtPcdViewer::on_selectFolderButton_clicked);
  connect(ui.fileTreeView, &QAbstractItemView::doubleClicked, this, &RqtPcdViewer::on_fileTreeView_doubleClicked);
  connect(ui.previousPcdButton, &QAbstractButton::clicked, this, &RqtPcdViewer::on_previousPcdButton_clicked);
  connect(ui.nextPcdButton, &QAbstractButton::clicked, this, &RqtPcdViewer::on_nextPcdButton_clicked);
  connect(ui.startPublishButton, &QAbstractButton::clicked, this, &RqtPcdViewer::startPublishing);
  connect(ui.stopPublishButton, &QAbstractButton::clicked, this, &RqtPcdViewer::stopPublishing);
  connect(ui.topicLineEdit, &QLineEdit::textChanged, this, &RqtPcdViewer::on_topicLineEdit_textChanged);
  connect(ui.visualizeCheckBox, &QCheckBox::toggled, this, &RqtPcdViewer::on_visualizeCheckBox_toggled);
  connect(&publishingTimer, &QTimer::timeout, this, &RqtPcdViewer::on_publishingTimer_timeout);

}

void RqtPcdViewer::shutdownPlugin()
{
  // unregister all publishers here
  if (folder_model)
    delete folder_model;
}

void RqtPcdViewer::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  instance_settings.setValue("lastFolder", settings.lastFolder);
}

void RqtPcdViewer::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  settings.lastFolder = instance_settings.value("lastFolder", QString()).toString();
  pluginSettingsToUi();
  applySettings();
}

bool RqtPcdViewer::hasConfiguration() const
{
  return true;
}

void RqtPcdViewer::triggerConfiguration()
{
}

void RqtPcdViewer::initializeViewer()
{
  ui.pcdView->setVisible(true);
  viewer.reset(new pcl::visualization::PCLVisualizer("PCD Viewer", false));
  ui.pcdView->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui.pcdView->GetInteractor(), ui.pcdView->GetRenderWindow());

  viewer->createViewPort(0.0, 0.0, 1.0, 1.0, viewport);
  viewer->setBackgroundColor (0, 0, 0, viewport);
  viewer->addText ("PC", 10, 10, "vpcap", viewport);

  ui.pcdView->update();
  viewer_initialized = true;
}

void RqtPcdViewer::deinitializeViewer()
{
  viewer->removeAllPointClouds(viewport);

  viewer.reset();
  viewer_initialized = false;

  ui.pcdView->setVisible(false);
}

void RqtPcdViewer::on_selectFolderButton_clicked()
{
  QString dir_path = QFileDialog::getExistingDirectory(widget, QString(), settings.lastFolder);
  if (dir_path.isEmpty())
    return;

  settings.lastFolder = dir_path;

  clearSelectedPcd();

  QFileSystemModel *model = new QFileSystemModel;
  model->setRootPath(dir_path);
  model->setNameFilterDisables(false);
  model->setNameFilters(QStringList("*.pcd"));
  ui.fileTreeView->setModel(model);
  ui.fileTreeView->setRootIndex(model->index(dir_path));
  ui.fileTreeView->hideColumn(1);
  ui.fileTreeView->hideColumn(2);
  ui.fileTreeView->hideColumn(3);

  if (folder_model)
    delete folder_model;

  folder_model = model;
}

void RqtPcdViewer::on_fileTreeView_doubleClicked(const QModelIndex &index)
{
  loadPcd(index);
}

void RqtPcdViewer::on_topicLineEdit_textChanged(const QString &topic)
{
  pointcloud_pub.shutdown();
  pointcloud_pub = getNodeHandle().advertise<sensor_msgs::PointCloud2>(topic.toStdString(), 1);
}

void RqtPcdViewer::on_visualizeCheckBox_toggled(bool checked)
{
  if (checked)
  {
    if (!viewer_initialized)
      initializeViewer();
  }
  else
  {
    if (viewer_initialized)
      deinitializeViewer();
  }
}

void RqtPcdViewer::on_previousPcdButton_clicked()
{
  if (!selected_pcd.isValid())
    return;

  int num_files_in_folder = folder_model->rowCount(selected_pcd.parent());
  int item_row = selected_pcd.row();

  auto previous_row = [&](int row) { return row > 0 ? row - 1 : num_files_in_folder - 1; };

  for (int cur_row = previous_row(item_row); cur_row != item_row; cur_row = previous_row(cur_row))
  {
    QModelIndex index = folder_model->index(cur_row, selected_pcd.column(), selected_pcd.parent());
    if (loadPcd(index))
      break;
  }
}

void RqtPcdViewer::on_nextPcdButton_clicked()
{
  if (!selected_pcd.isValid())
    return;

  int num_files_in_folder = folder_model->rowCount(selected_pcd.parent());
  int item_row = selected_pcd.row();

  auto next_row = [&](int row) { return (row + 1) % num_files_in_folder; };

  for (int cur_row = next_row(item_row); cur_row != item_row; cur_row = next_row(cur_row))
  {
    QModelIndex index = folder_model->index(cur_row, selected_pcd.column(), selected_pcd.parent());
    if (loadPcd(index))
      break;
  }
}

void RqtPcdViewer::on_publishingTimer_timeout()
{
  if (ui.rotateFilesCheckBox->isChecked())
  {
    if (ui.rotateBackwardsCheckbox->isChecked())  
      on_previousPcdButton_clicked();
    else
      on_nextPcdButton_clicked();

    publishPointcloud();
  }
  else if (ui.publishContinouslyCheckBox->isChecked())
  {
    publishPointcloud();
  }
}

void RqtPcdViewer::startPublishing()
{
  publishPointcloud();
  if (ui.publishContinouslyCheckBox->isChecked() || ui.rotateFilesCheckBox->isChecked())
  {
    publishingTimer.start(1000.0 / ui.frequencySpinBox->value());
  }
}

void RqtPcdViewer::stopPublishing()
{
  publishingTimer.stop();
}

bool RqtPcdViewer::loadPcd(const QModelIndex &index)
{
  if (folder_model->isDir(index))
  {
    ROS_INFO_STREAM("Selection is directory");
    return false;
  }

  QString path = folder_model->filePath(index);

  cloud.reset(new pcl::PCLPointCloud2);

  try
  {
    pcl::io::loadPCDFile(path.toStdString(), *cloud);
  }
  catch (const pcl::io::IOException &e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  if (viewer_initialized)
  {
    visualizePointcloud();
  }

  setSelectedPcd(index);

  ui.previousPcdButton->setEnabled(true);
  ui.nextPcdButton->setEnabled(true);
  pcd_loaded = true;

  return true;
}

void RqtPcdViewer::visualizePointcloud()
{
  /* Color Handlers:
   * PointCloudColorHandlerRandom (const PointCloudConstPtr &cloud)
   * PointCloudColorHandlerCustom (const PointCloudConstPtr &cloud, double r, double g, double b)
   * PointCloudColorHandlerRGBField (const PointCloudConstPtr &cloud);
   * PointCloudColorHandlerHSVField (const PointCloudConstPtr &cloud);
   * PointCloudColorHandlerGenericField (const PointCloudConstPtr &cloud, const std::string &field_name);
   * PointCloudColorHandlerRGBAField (const PointCloudConstPtr &cloud);
   * PointCloudColorHandlerLabelField (const PointCloudConstPtr &cloud, const bool static_mapping = true);
   */
  pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr cloud_ch;
  std::unordered_set<std::string> field_names;
  for (const auto &field : cloud->fields) field_names.insert(field.name);
  if (field_names.find("label") != field_names.end())
  {
    cloud_ch.reset(new pcl::visualization::PointCloudColorHandlerLabelField<pcl::PCLPointCloud2>(cloud));
  }
  else if (field_names.find("rgba") != field_names.end())
  {
    cloud_ch.reset(new pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PCLPointCloud2>(cloud));
  }
  else if (field_names.find("hsv") != field_names.end())
  {
    cloud_ch.reset(new pcl::visualization::PointCloudColorHandlerHSVField<pcl::PCLPointCloud2>(cloud));
  }
  else if (field_names.find("rgb") != field_names.end())
  {
    cloud_ch.reset(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(cloud));
  }
  else
  {
    cloud_ch.reset(new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(cloud));
  }

  pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>::Ptr cloud_gh(new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>(cloud));

  const Eigen::Vector4f sensor_origin;
  const Eigen::Quaternion<float> sensor_orientation;

  viewer->removeAllPointClouds(viewport);
  viewer->addPointCloud(cloud, cloud_gh, cloud_ch, sensor_origin, sensor_orientation, "pc", viewport);
  viewer->initCameraParameters();
  viewer->resetCameraViewpoint("pc");
  viewer->resetCamera();
  viewer->spinOnce(1, true);
  ui.pcdView->update();
}

void RqtPcdViewer::publishPointcloud()
{
  if (!pcd_loaded)
    return;

  sensor_msgs::PointCloud2 msg;
  pcl_conversions::fromPCL(*cloud, msg);
  msg.header.frame_id = ui.frameLineEdit->text().toStdString();
  msg.header.stamp = ros::Time::now();
  pointcloud_pub.publish(msg);
}

void RqtPcdViewer::setSelectedPcd(QModelIndex index)
{
  selected_pcd = index;
  int num_imgs_in_folder = folder_model->rowCount(selected_pcd.parent());
  int item_row = selected_pcd.row();
  ui.curPcdLabel->setText(QString::asprintf("%d/%d", item_row + 1, num_imgs_in_folder));
  ui.fileTreeView->setCurrentIndex(selected_pcd);
}

void RqtPcdViewer::clearSelectedPcd()
{
  if (viewer_initialized)
  {
    viewer->removeAllPointClouds(viewport);
    viewer->spinOnce(1, true);
    ui.pcdView->update();
  }
  

  selected_pcd = QModelIndex();
  pcd_loaded = false;
  ui.previousPcdButton->setEnabled(false);
  ui.nextPcdButton->setEnabled(false);
  ui.curPcdLabel->setText("0/0");
  ui.fileTreeView->clearSelection();
}

void RqtPcdViewer::pluginSettingsToUi()
{
}

void RqtPcdViewer::uiToPluginSettings()
{
}

void RqtPcdViewer::applySettings()
{
}

} // namespace rqt_pcd_viewer

PLUGINLIB_EXPORT_CLASS(rqt_pcd_viewer::RqtPcdViewer, rqt_gui_cpp::Plugin)
//PLUGINLIB_DECLARE_CLASS(rqt_pcd_viewer, RqtPcdViewer, rqt_pcd_viewer::RqtPcdViewer, rqt_gui_cpp::Plugin)
