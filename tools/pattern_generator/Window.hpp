#ifndef WINDOW_HPP
#define WINDOW_HPP

#include <QtWidgets>

class Window : public QWidget
{
    Q_OBJECT

public:
    Window(QWidget *parent = 0);
    void Convert();
    void setFrameReduction(int value);
    void setEndOff(bool value);
    void setVideoPath(QString path) { mVideoPath = path; }
    void setOutputFile(QString path) { mOutputFile = fopen((char*)path.toStdString().c_str(), "w"); }

public slots:
  void SelectVideoSlot(); // Select the video
  void ConvertVideoSlot(); // Start conversion

protected:

  // Store the rgb value extract from pixel (x,y) into the output file
  // if x or y == -1, both r, g and b are set to 0
  void StoreColorLed(int x, int y);

private:
  QVBoxLayout  *mVlayout;

  QPushButton  *mSelectVideoButton;
  QPushButton  *mConvertVideoButton;

  QHBoxLayout  *mHlayout;
  QGroupBox    *mOptionsGroupBox;
  QRadioButton *mFaceRadioButton;
  QRadioButton *mCornerRadioButton;
  QRadioButton *mBothRadioButton;
  QLabel       *mLabel;

  QCheckBox    *mCheckBox;

  QSpinBox     *mFrameReductionSpinbox;

  QImage        mImage;
  QString       mVideoPath;

  FILE         *mOutputFile;

  static const int CornerXOffset[4*6];
  static const int CornerYOffset[4*6];
  static const int PanelYOffset[9];
  static const int PanelXOffset[3];

};

#endif // WINDOW_HPP
