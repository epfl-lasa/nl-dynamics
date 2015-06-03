#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <getopt.h>

#include "GPMDS.h"
#include "Timer.h"


using namespace std;
using namespace Eigen;

void load_training_data(const char *fname, vector<Vector3r> &pos, vector<Vector3r> &vel) {
  Vector3r tempPos;
  Vector3r tempVel;
  ifstream myfile;
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

int main2(int argc, char *argv[]) {
  const double ell = 0.07;
  const double sigmaF = 1.0;
  const double sigmaN = 0.4;
  const double speedErrorTol = 0.01;
  const double angleErrorTol = 0.1;
  GPMDS mGPMDS(ell, sigmaF, sigmaN, speedErrorTol, angleErrorTol);
  mGPMDS.setOriginalDynamics(&linear_isotropic_dynamics);
  vector<Vector3r> training_pos;
  vector<Vector3r> training_vel;

  load_training_data("data.txt", training_pos, training_vel);
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

static int verbose_flag;
int
main (int argc, char **argv)
{
  int c;
  string filename = "data.txt";

  while (1)
    {
      static struct option long_options[] =
        {
          /* These options set a flag. */
          {"verbose", no_argument,       &verbose_flag, 1},
          /* These options don’t set a flag.
             We distinguish them by their indices. */
          {"file",    required_argument, 0, 'f'},
          {0}
        };
      /* getopt_long stores the option index here. */
      int option_index = 0;

      c = getopt_long (argc, argv, "f:",
                       long_options, &option_index);

      /* Detect the end of the options. */
      if (c == -1)
        break;

      switch (c)
        {
        case 0:
          /* If this option set a flag, do nothing else now. */
          if (long_options[option_index].flag != 0)
            break;
          printf ("option %s", long_options[option_index].name);
          if (optarg)
            printf (" with arg %s", optarg);
          printf ("\n");
          break;

        case 'f':
          printf ("option -f with value `%s'\n", optarg);
          filename = optarg;
          break;

        case '?':
          /* getopt_long already printed an error message. */
          break;

        default:
          abort ();
        }
    }

  /* Instead of reporting ‘--verbose’
     and ‘--brief’ as they are encountered,
     we report the final status resulting from them. */
  if (verbose_flag)
    puts ("verbose flag is set");

  /* Print any remaining command line arguments (not options). */
  if (optind < argc)
    {
      printf ("non-option ARGV-elements: ");
      while (optind < argc)
        printf ("%s ", argv[optind++]);
      putchar ('\n');
    }

  cout << "Filename: " << filename << endl;

  exit (0);
}
