#include "rqt_pcd_viewer/rqt_pcd_viewer.h"

#include <pluginlib/class_list_macros.h>
#include <QFileDialog>
#include <QInputDialog>
#include <pcl/io/io_exception.h>

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
  ui.pcdView->update();

  int vp1, vp2;

  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, vp1);
  viewer->setBackgroundColor (0, 0, 0, vp1);
  viewer->addText ("PC1", 10, 10, "vp1cap", vp1);
  vp_map[0] = vp1;

  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp2);
  viewer->setBackgroundColor (0.1, 0.1, 0.1, vp2);
  viewer->addText ("PC2", 10, 10, "vp2cap", vp2);
  vp_map[1] = vp2;

  ROS_INFO_STREAM("Viewports: " << vp1 << ", " << vp2);

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
}

void RqtPcdViewer::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
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
  QString dir_path = QFileDialog::getExistingDirectory(widget, QString());
  if (dir_path.isEmpty())
    return;

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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  try
  {
    pcl::io::loadPCDFile(path.toStdString(), *cloud);
  }
  catch (const pcl::io::IOException &e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  viewer->removeAllPointClouds(vp_map[vpind]);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "pc" + std::to_string(vpind), vp_map[vpind]);
  viewer->spinOnce(1, true);

  setSelectedPcd(index, vpind);

  previousPcdButton[vpind]->setEnabled(true);
  nextPcdButton[vpind]->setEnabled(true);
  pcd_loaded[vpind] = true;
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
