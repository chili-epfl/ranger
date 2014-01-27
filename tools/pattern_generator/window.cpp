#include "Window.hpp"

// Y offset of each of the Lower Led of a panel
const int Window::PanelYOffset[9] = { 128, 139, 128, 139, 128, 139, 128, 139, 128 };
const int Window::PanelXOffset[3] = { 478, 258, 38 };

// Corner and Pannel order 
const int cornerOrder[4] = { 1, 0, 3, 2};
const int pannelOrder[3] = { 0, 1, 2};

Window::Window(QWidget *parent) : QWidget(parent)
{
  mVlayout = new QVBoxLayout(this);
  mVideoPath = QString("");

  // Selection Button
  mSelectVideoButton = new QPushButton(tr("Select a Video"), this);
  mVlayout->addWidget(mSelectVideoButton);
  QObject::connect(mSelectVideoButton, SIGNAL(clicked()), this, SLOT(SelectVideoSlot()));

  // Options
  mHlayout = new QHBoxLayout();
  mOptionsGroupBox = new QGroupBox(tr("Options"), this);
  mOptionsGroupBox->setToolTip(tr("Select wich LEDs you want to use."));
  mFaceRadioButton = new QRadioButton(tr("Face"), this);
  mCornerRadioButton = new QRadioButton(tr("Edge"), this);
  mBothRadioButton = new QRadioButton(tr("Both"), this);
  mBothRadioButton->setChecked(true);

  mHlayout->addWidget(mFaceRadioButton);
  mHlayout->addWidget(mCornerRadioButton);
  mHlayout->addWidget(mBothRadioButton);
  mOptionsGroupBox->setLayout(mHlayout);
  mVlayout->addWidget(mOptionsGroupBox);

  mCheckBox = new QCheckBox("Switch off each leds at end");
  mCheckBox->setChecked(false);
  mVlayout->addWidget(mCheckBox);

  mFrameReductionSpinbox = new QSpinBox(this);
  mFrameReductionSpinbox->setMinimum(1);
  mFrameReductionSpinbox->setPrefix("Frame reduction:  ");
  mFrameReductionSpinbox->setToolTip(tr("If the video contains too much frames,\nIt is possible to reduce them by increasing the frame reduction factors."));
  mVlayout->addWidget(mFrameReductionSpinbox);

  mLabel = new QLabel(this);
  mVlayout->addWidget(mLabel);

  // Conversion Button
  mConvertVideoButton = new QPushButton(tr("Convert Video"), this);
  mVlayout->addWidget(mConvertVideoButton);
  QObject::connect(mConvertVideoButton, SIGNAL(clicked()), this, SLOT(ConvertVideoSlot()));

  setWindowTitle(tr("Ranger Video Converter"));
}

void Window::SelectVideoSlot() {
  mVideoPath = QFileDialog::getOpenFileName(this, tr("Choose a video"), QString(), "Videos (*.mp4)");
}

void Window::ConvertVideoSlot() {
  // Check that a video has first been selected
  if(mVideoPath.isEmpty()) {
    QMessageBox::warning(this, tr("No video selected"), tr("You must first select a video to convert"));
    return;
  }

  // Select output file
  QString OutputFile = QFileDialog::getSaveFileName(this, tr("Save file"), QString(), "Video pattern (*.vid)");

  if(!OutputFile.isEmpty()) {
    mOutputFile = fopen((char*)OutputFile.toStdString().c_str(), "wb");
    Convert();
  }
}

void Window::Convert() {

  // create frame images in temporary files
  QDir directory(QDir::tempPath() + "/Ranger");

  //directory.removeRecursively();

  QStringList args;
  args << "-rf" << QDir::tempPath() + "/Ranger";
  QProcess::execute("rm",args);

  directory.mkdir(QDir::tempPath() + "/Ranger");

  QString program = "ffmpeg";
  QStringList arguments;
  arguments << "-i" << mVideoPath << "-f" << "image2" << QDir::tempPath() + "/Ranger/frame%04d.png";
  QProcess::execute(program, arguments);
  QFileInfoList FrameList = directory.entryInfoList();
  int FrameNumber = FrameList.size() - 2; // -2 for '.' and '..'


  for(int i = 1; i < (FrameNumber+1); i+=mFrameReductionSpinbox->value()) { // Foreach frame

    // Load Image Frame
    mImage.load(QDir::tempPath() + QString().sprintf("/Ranger/frame%04d.png",i));

    // Extracte Led Colors
    for(int c1=0; c1 < 4; ++c1) {

      // Corner
      for(int c2=0; c2 < 6; ++c2) {
        if(mBothRadioButton->isChecked() || mCornerRadioButton->isChecked())
          StoreColorLed(8 + cornerOrder[c1] * 223, 18 + 22 * c2);
        else
          StoreColorLed(-1, -1);
      }
      // panel
      if(c1<3) {
        for(int c2=0; c2<9; ++c2) {
          for(int c3=0; c3<6; ++c3) {
            if(mBothRadioButton->isChecked() || mFaceRadioButton->isChecked()) {
              if(c2 & 0x1) // Odd column
                StoreColorLed(PanelXOffset[pannelOrder[c1]] + 21 * c2, PanelYOffset[c2] - c3 * 22);
              else
                StoreColorLed(PanelXOffset[pannelOrder[c1]] + 21 * c2, PanelYOffset[c2] - (5-c3) * 22);
            }
            else
              StoreColorLed(-1, -1);
          }
        }
      }
    }
  }

  // switch off each leds
  if(mCheckBox->isChecked()) {
    for(int i = 0; i < 186; ++i)
      StoreColorLed(-1, -1);
  }

  fclose(mOutputFile);
  QProcess::execute("rm",args);
}

void Window::StoreColorLed(int x, int y) {
  unsigned char rgb[3] = {0,0,0};

  if(x != -1 && y != -1) {
    QRgb color = mImage.pixel(x, y);
    rgb[0] = qGreen(color)/2 | 0x80;              // /2 => 255->127
    rgb[1] = qRed(color)/2 | 0x80;
    rgb[2] = qBlue(color)/2 | 0x80;
  }
  fwrite(rgb,3,1,mOutputFile);
}

void Window::setFrameReduction(int value) {
  if(value>1)
    mFrameReductionSpinbox->setValue(value);
}

void Window::setEndOff(bool value) {
  mCheckBox->setChecked(value);
}
