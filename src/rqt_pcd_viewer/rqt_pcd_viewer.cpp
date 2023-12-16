#include "rqt_pcd_viewer/rqt_pcd_viewer.h"

#include <pluginlib/class_list_macros.h>
#include <QFileDialog>
#include <QInputDialog>
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
  widget(Q_NULLPTR),
  viewer(Q_NULLPTR)
{
  for (size_t i=0; i < NUM_VIEWS; i++)
  {
    folder_model[i] = nullptr;
    pcd_loaded[i] = false;
    selected_pcd[i] = QModelIndex();
    fileTreeView[i] = nullptr;
    selectFolderButton[i] = nullptr;
    previousPcdButton[i] = nullptr;
    curPcdLabel[i] = nullptr;
    nextPcdButton[i] = nullptr;
  }
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

  fileTreeView[0] = ui.fileTreeViewLeft;
  fileTreeView[1] = ui.fileTreeViewRight;
  selectFolderButton[0] = ui.selectFolderButtonLeft;
  selectFolderButton[1] = ui.selectFolderButtonRight;
  previousPcdButton[0] = ui.previousPcdButtonLeft;
  previousPcdButton[1] = ui.previousPcdButtonRight;
  curPcdLabel[0] = ui.curPcdLabelLeft;
  curPcdLabel[1] = ui.curPcdLabelRight;
  nextPcdButton[0] = ui.nextPcdButtonLeft;
  nextPcdButton[1] = ui.nextPcdButtonRight;

  viewer.reset(new pcl::visualization::PCLVisualizer("PCD Viewer", false));
  ui.pcdView->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui.pcdView->GetInteractor(), ui.pcdView->GetRenderWindow());

  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, viewport[0]);
  viewer->setBackgroundColor (0, 0, 0, viewport[0]);
  viewer->addText ("PC1", 10, 10, "vp1cap", viewport[0]);

  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, viewport[1]);
  viewer->setBackgroundColor (0.1, 0.1, 0.1, viewport[1]);
  viewer->addText ("PC2", 10, 10, "vp2cap", viewport[1]);

  ui.pcdView->update();

  for (size_t i=0; i < NUM_VIEWS; i++)
  {
    connect(selectFolderButton[i], &QAbstractButton::clicked, this, boost::bind(&RqtPcdViewer::on_selectFolderButton_clicked, this, i));
    connect(fileTreeView[i], &QAbstractItemView::doubleClicked, this, boost::bind(&RqtPcdViewer::on_fileTreeView_doubleClicked, this, _1, i));
    connect(previousPcdButton[i], &QAbstractButton::clicked, this, boost::bind(&RqtPcdViewer::on_previousPcdButton_clicked, this, i));
    connect(nextPcdButton[i], &QAbstractButton::clicked, this, boost::bind(&RqtPcdViewer::on_nextPcdButton_clicked, this, i));
  }

}

void RqtPcdViewer::shutdownPlugin()
{
  // unregister all publishers here
  for (size_t i=0; i < NUM_VIEWS; i++)
  {
    if (folder_model[i])
      delete folder_model[i];
  }
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

void RqtPcdViewer::on_selectFolderButton_clicked(size_t vpind)
{
  QString dir_path = QFileDialog::getExistingDirectory(widget, QString(), settings.lastFolder);
  if (dir_path.isEmpty())
    return;

  settings.lastFolder = dir_path;

  clearSelectedPcd(vpind);

  QFileSystemModel *model = new QFileSystemModel;
  model->setRootPath(dir_path);
  model->setNameFilterDisables(false);
  model->setNameFilters(QStringList("*.pcd"));
  fileTreeView[vpind]->setModel(model);
  fileTreeView[vpind]->setRootIndex(model->index(dir_path));
  fileTreeView[vpind]->hideColumn(1);
  fileTreeView[vpind]->hideColumn(2);
  fileTreeView[vpind]->hideColumn(3);

  if (folder_model[vpind])
    delete folder_model[vpind];

  folder_model[vpind] = model;
}

void RqtPcdViewer::on_fileTreeView_doubleClicked(const QModelIndex &index, size_t vpind)
{
  loadPcd(index, vpind);
}

void RqtPcdViewer::on_previousPcdButton_clicked(size_t vpind)
{
  if (!selected_pcd[vpind].isValid())
    return;

  int num_imgs_in_folder = folder_model[vpind]->rowCount(selected_pcd[vpind].parent());
  int item_row = selected_pcd[vpind].row();

  auto previous_row = [&](int row) { return row > 0 ? row - 1 : num_imgs_in_folder - 1; };

  for (int cur_row = previous_row(item_row); cur_row != item_row; cur_row = previous_row(cur_row))
  {
    QModelIndex index = folder_model[vpind]->index(cur_row, selected_pcd[vpind].column(), selected_pcd[vpind].parent());
    if (loadPcd(index, vpind))
      break;
  }
}

void RqtPcdViewer::on_nextPcdButton_clicked(size_t vpind)
{
  if (!selected_pcd[vpind].isValid())
    return;

  int num_imgs_in_folder = folder_model[vpind]->rowCount(selected_pcd[vpind].parent());
  int item_row = selected_pcd[vpind].row();

  auto next_row = [&](int row) { return (row + 1) % num_imgs_in_folder; };

  for (int cur_row = next_row(item_row); cur_row != item_row; cur_row = next_row(cur_row))
  {
    QModelIndex index = folder_model[vpind]->index(cur_row, selected_pcd[vpind].column(), selected_pcd[vpind].parent());
    if (loadPcd(index, vpind))
      break;
  }
}

bool RqtPcdViewer::loadPcd(const QModelIndex &index, size_t vpind)
{
  if (folder_model[vpind]->isDir(index))
  {
    ROS_INFO_STREAM("Selection is directory");
    return false;
  }

  QString path = folder_model[vpind]->filePath(index);

  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);

  try
  {
    pcl::io::loadPCDFile(path.toStdString(), *cloud);
  }
  catch (const pcl::io::IOException &e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

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

  viewer->removeAllPointClouds(viewport[vpind]);
  viewer->addPointCloud(cloud, cloud_gh, cloud_ch, sensor_origin, sensor_orientation, "pc" + std::to_string(vpind), viewport[vpind]);
  viewer->initCameraParameters();
  viewer->resetCameraViewpoint("pc" + std::to_string(vpind));
  viewer->resetCamera();
  viewer->spinOnce(1, true);
  ui.pcdView->update();

  setSelectedPcd(index, vpind);

  previousPcdButton[vpind]->setEnabled(true);
  nextPcdButton[vpind]->setEnabled(true);
  pcd_loaded[vpind] = true;

  ROS_INFO_STREAM("New PCD loaded");

  return true;
}

void RqtPcdViewer::setSelectedPcd(QModelIndex index, size_t vpind)
{
  selected_pcd[vpind] = index;
  int num_imgs_in_folder = folder_model[vpind]->rowCount(selected_pcd[vpind].parent());
  int item_row = selected_pcd[vpind].row();
  curPcdLabel[vpind]->setText(QString::asprintf("%d/%d", item_row + 1, num_imgs_in_folder));
  fileTreeView[vpind]->setCurrentIndex(selected_pcd[vpind]);
}

void RqtPcdViewer::clearSelectedPcd(size_t vpind)
{
  viewer->removeAllPointClouds(viewport[vpind]);
  viewer->spinOnce(1, true);
  ui.pcdView->update();

  selected_pcd[vpind] = QModelIndex();
  pcd_loaded[vpind] = false;
  previousPcdButton[vpind]->setEnabled(false);
  nextPcdButton[vpind]->setEnabled(false);
  curPcdLabel[vpind]->setText("0/0");
  fileTreeView[vpind]->clearSelection();
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
