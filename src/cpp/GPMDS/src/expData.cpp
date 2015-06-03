#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <getopt.h>

#include "GPMDS.h"
#include "Timer.h"


using namespace std;
using namespace Eigen;

// Forward declaration
bool parse_args(int argc, char **argv, string *filename, int *verbose_flag);

void load_training_data(const char *fname, vector<Vector3r> &pos, vector<Vector3r> &vel) {
  Vector3r tempPos;
  Vector3r tempVel;
  ifstream myfile;

  cout << "Loading training data from " << fname << endl;
  myfile.open(fname);
  while(myfile >> tempPos(0)){
    myfile>>tempPos(1)>>tempPos(2)>>tempVel(0)>>tempVel(1)>>tempVel(2);
    pos.push_back(tempPos);
    vel.push_back(tempVel);
  }
}

Vector3r linear_isotropic_dynamics(Vector3r pos) {
  Vector3r vel;
  vel = -3*pos;
  return vel;
}

int main(int argc, char *argv[]) {

  string filename = "data.txt";
  int verbose_flag = 0;

  bool ret = parse_args(argc, argv, &filename, &verbose_flag);
  if (!ret)
    return 0;

  const double ell = 0.07;
  const double sigmaF = 1.0;
  const double sigmaN = 0.4;
  const double speedErrorTol = 0.01;
  const double angleErrorTol = 0.1;
  GPMDS mGPMDS(ell, sigmaF, sigmaN, speedErrorTol, angleErrorTol);
  mGPMDS.setOriginalDynamics(&linear_isotropic_dynamics);
  vector<Vector3r> training_pos;
  vector<Vector3r> training_vel;

  load_training_data(filename.c_str(), training_pos, training_vel);
  cout<<"done reading "<<training_pos.size()<<" "<<training_vel.size()<<endl;
  Timer tmr;
  int N = training_pos.size();
  for (int i = 0; i < N; i++){
    mGPMDS.addData(training_pos[i], training_vel[i]);
  }
  cout<<"done adding "<<tmr.elapsed()<<endl;

  tmr.reset();
  mGPMDS.prepareFastQuery();
  cout<<"preparing took.."<<tmr.elapsed()<<endl;
  Vector3r tvel;
  tmr.reset();
  tvel = mGPMDS.reshapedDynamics(training_pos[0]);
  cout<<"reshaping took.."<<tmr.elapsed()<<endl;

  return 0;
}

static void
usage (const char *progname){

    fprintf (stdout, "Usage: %s [options]\n"
             "\n"
             "Options:\n"
             "    -f, --file NAME        Data file name\n"
             "    --verbose              Verbose\n"
             "    -h, --help             This help message\n"
             "\n", progname);

    return;
}

bool parse_args(int argc, char **argv,
                string *filename,
                int *verbose_flag) {
  int c;
  while (1) {
    static struct option long_options[] = {
      {"verbose", no_argument, verbose_flag, 1},
      {"file", required_argument, 0, 'f'},
      {"help", no_argument, 0, 'h'},
      {0, 0}
    };
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long (argc, argv, "vf:h", long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1)
      break;

    switch (c) {
    case 0:
      /* If this option set a flag, do nothing else now. */
      if (long_options[option_index].flag != 0)
        break;
      break;

    case 'f':
      *filename = optarg;
      break;

    case 'h':
      cout << "help" << endl;
      usage(argv[0]);
      return false;
      break;

    case '?':
      /* getopt_long already printed an error message. */
      break;

    default:
      abort ();
    }
  }

  /* Print any remaining command line arguments (not options). */
  if (optind < argc) {
    printf ("other arguments: ");
    while (optind < argc)
      printf ("%s ", argv[optind++]);
    putchar ('\n');
  }

  return true;
}
