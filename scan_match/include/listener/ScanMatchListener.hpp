

class ScanMatchListener {
 public:
  ScanMatchListener();
  ~ScanMatchListener();

  virtual void setData();
};

class Listener1 {
 public:
  Listener1();
  ~Listener1();

  void setData(pcl_cloud);
};

class Listener2 {
 public:
  Listener2();
  ~Listener2();

  void setData(eigen_cloud);
};