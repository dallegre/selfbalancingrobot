class calc{
  public:
    int sigint;
    int sigprev;
    int derivative(int sig);
    int integral(int sig);
    calc();
};

calc::calc(void){
  sigprev = 0;
  sigint =  0;
}

int calc::derivative(int sig){
  int der = sig - sigprev;
  sigprev = sig;
  return der;
}

int calc::integral(int sig){
  sigint += sig;
  return sigint;
}

