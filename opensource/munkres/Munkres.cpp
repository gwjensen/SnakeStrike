

/* Note that this algorithm implicitely expects the values of the costMatrix to be > 0, i.e. no negative numbers. - GWJ */

/* This code is from
https://github.com/georgepar/munkres and is licensed under GNU General Public License v3.0
*/

#include "Munkres.h"
#include <utility>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <climits>
//The implementation is based on the implementation proposed in
//http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html
//This is the O(n^3) version of the algorithm

using namespace Eigen;

namespace opensource
{

  Munkres::Munkres(const Array<int, Dynamic, Dynamic, RowMajor>& cost_matrix)
  {
    N_ = (cost_matrix.rows() > cost_matrix.cols()) ? cost_matrix.rows() : cost_matrix.cols();
    //C_ = cost_matrix;
    C_.resize(N_, N_);
    pad_matrix(cost_matrix, C_);
    row_cover_ = std::vector<bool>(N_, false);
    col_cover_ = std::vector<bool>(N_, false);
    Z0_ = std::make_pair(0,0);
    starred_ =  Array<int, Dynamic, Dynamic, RowMajor>::Zero(N_, N_);
    path_ =  Array<int, Dynamic, 2, RowMajor>::Zero(N_*2, 2);
  }

  void Munkres::pad_matrix(const Array<int, Dynamic, Dynamic, RowMajor>& cost_matrix,
                  Array<int, Dynamic, Dynamic, RowMajor>& C)
  {
    int crows = cost_matrix.rows();
    int ccols = cost_matrix.cols();
    int N = (crows > ccols) ? crows : ccols;
    Array<int, Dynamic, Dynamic, RowMajor> padded(N, N);
    for(int i = 0; i < N; i++) {
        for(int j = 0; j < N; j++) {
            if(i < crows && j < ccols) {
                padded(i, j) = cost_matrix(i, j);
            }
            else {
                padded(i, j) = INT_MAX;
            }
        }
    }
    C = padded;
  }

  std::vector<std::pair<int, int> > Munkres::run()
  {


    bool done = false;
    int step = 1;

    while(!done) {
        //try printing out all the matrices and see if anything shows up - GWJ
        //std::cout << "C" << std::endl << C_ << std::endl;
        //std::cout << "starred_" << std::endl << starred_ << std::endl;
        //std::cout << "path_" << std::endl  << path_ << std::endl;
      switch(step) {
        case 1:
          step = step1();
          break;
        case 2:
          step = step2();
          break;
        case 3:
          step = step3();
          break;
        case 4:
          step = step4();
          break;
        case 5:
          step = step5();
          break;
        case 6:
          step = step6();
          break;
        default:
          done = true;
          break;
      }
    }
    std::vector< std::pair<int, int> > matches(N_);

    for(int i = 0; i < N_; i++) {
      for(int j = 0; j < N_; j++) {
        if(starred_(i,j) == 1) {
          matches[i] = std::make_pair(i, j);
        }
      }
    }

    return matches;
  }

  //subtract from every row of C_ the minimum value of that row
  int Munkres::step1()
  {
    for(int i = 0; i < N_; i++) {
      int min_r = (C_.row(i)).minCoeff();
      if(min_r < INT_MAX) {
        C_.row(i) -=  min_r;
      }
    }
    return 2;
  }

  //"star" a zero if it is the only zero in that row or column
  //starred zeros are possible valid matches
  int Munkres::step2()
  {
    for(int i = 0; i < N_; i++) {
      for(int j = 0; j < N_; j++) {
        if(C_(i,j) == 0 && !row_cover_[i] && !col_cover_[j]) {
          starred_(i,j) = 1;
          row_cover_[i] = true;
          col_cover_[j] = true;
        }
      }
    }
    clear_covers();
    return 3;
  }

  //cover all columns containing a starred zero.
  //if every column is covered we are done
  //else go to step4
  int Munkres::step3()
  {
    int cols_covered = 0;
    for(int i = 0; i < N_; i++) {
      for(int j = 0; j < N_; j++) {
        if(starred_(i,j) == 1) {
          col_cover_[j] = true;
          cols_covered++;
        }
      }
    }

    int step;
    if(cols_covered >= N_) {
      step = 7;
    }
    else {
      step = 4;
    }
    return step;
  }

  //Find a noncovered zero and prime it.  If there is no starred zero in the row
  //containing this primed zero, Go to Step 5.  Otherwise, cover this row
  //and uncover the column containing the starred zero. Continue in this manner
  //until there are no uncovered zeros left. Save the smallest uncovered value
  //and Go to Step 6.
  int Munkres::step4()
  {
    int row = -1;
    int col = -1;
    int col_star = -1;
    bool done = false;
    int step;

    while(!done) {
      std::pair<int, int> Z = find_a_zero();
      row = Z.first;
      col = Z.second;
      if(row < 0) {
        done = true;
        step = 6;
      }
      else {
        starred_(row, col) = 2;
        col_star = find_star_in_row(row);
        if(col_star >= 0) {
          col = col_star;
          row_cover_[row] = true;
          col_cover_[col] = false;
        }
        else {
          done = true;
          step = 5;
          Z0_.first = row;
          Z0_.second = col;
        }
      }
    }
    return step;
  }

  //Construct a series of alternating primed and starred zeros as follows.
  //Let Z0 represent the uncovered primed zero found in Step 4.
  //Let Z1 denote the starred zero in the column of Z0 (if any).
  //Let Z2 denote the primed zero in the row of Z1 (there will always be one).
  //Continue until the series terminates at a primed zero that has no starred zero
  //in its column.  Unstar each starred zero of the series, star each primed zero
  //of the series, erase all primes and uncover every line in the matrix.
  //Return to Step 3.
  int Munkres::step5()
  {
    bool done = false;
    int row = -1;
    int col = -1;
    int path_count = 0;
    //std::cout << "starting loop" << std::endl << "beforeZ:path_" << std::endl << path_ << std::endl;
    path_(path_count, 0) = Z0_.first;
    path_(path_count, 1) = Z0_.second;
    //std::cout << "afterZ:path_" << std::endl << path_ << std::endl;
    //std::cout << "path_count = " << path_count << std::endl;
    while(!done) {
      //std::cout << "find_col:starred_" << std::endl << starred_ << std::endl;
      row = find_star_in_col(path_(path_count, 1));
      //std::cout << "row = " << std::endl << row << std::endl;
      if(row > -1) {
        path_count++;
        path_(path_count, 0) = row;
        path_(path_count, 1) = path_(path_count - 1, 1);
        //std::cout << "path_count = " << path_count << std::endl;
        //std::cout << "p1:path_" << std::endl << path_ << std::endl;
      }
      else {
        done = true;
      }
      if(!done) {
        //std::cout << "findprime:starred_" << std::endl << starred_ << std::endl;
        col = find_prime_in_row(path_(path_count, 0));
        //std::cout << "col = " << std::endl << col << std::endl;

        path_count++;///@todo
        path_(path_count, 0) = path_(path_count - 1, 0);///@todo
        path_(path_count, 1) = col;
        //std::cout << "path_count = " << path_count << std::endl;
        //std::cout << "p2:path_" << std::endl << path_ << std::endl;
      }
    }
    augment_path(path_count);
    clear_covers();
    erase_primes();
    return 3;
  }

  //Add the value found in Step 4 to every element of each covered row,
  //and subtract it from every element of each uncovered column.
  //Return to Step 4 without altering any stars, primes, or covered lines.
  int Munkres::step6()
  {
    int epsilon = find_epsilon();
    for(int i = 0; i < N_; i++) {
      for(int j = 0; j < N_; j++) {
        if(row_cover_[i]) {
          C_(i, j) += epsilon;
        }
        if(!col_cover_[j]) {
          C_(i, j) -= epsilon;
        }
      }
    }

    return 4;
  }

  void Munkres::clear_covers()
  {
    std::fill(row_cover_.begin(), row_cover_.end(), false);
    std::fill(col_cover_.begin(), col_cover_.end(), false);
  }

  std::pair<int, int> Munkres::find_a_zero()
  {
    int row = -1;
    int col = -1;
    bool done = false;
    for(int c = 0; c < N_ && !done; c++) {
      for(int r = 0; r < N_ && !done; r++) {
        if((C_(r, c) == 0) && !row_cover_[r] && !col_cover_[c]) {
          row = r;
          col = c;
          done = true;
        }
      }
    }
    return std::make_pair(row, col);
  }

  int Munkres::find_star_in_row(int row)
  {
    int col = -1;
    for(int j = 0; j < N_; j++) {
      if(starred_(row, j) == 1) {
        col = j;
        break;
      }
    }
    return col;
  }

  int Munkres::find_star_in_col(int col)
  {
    int row = -1;
    for(int i = 0; i < N_; i++) {
      if(starred_(i, col) == 1) {
        row = i;
        break;
      }
    }
    return row;
  }

  int Munkres::find_prime_in_row(int row)
  {
    int col = -1;
    for(int j = 0; j < N_; j++) {
      if(starred_(row, j) == 2) {
        col = j;
        break;
      }
    }
    return col;
  }

  void Munkres::augment_path( int path_count)
  {
    for(int p = 0; p < path_count+1; p++) {
      if(starred_(path_(p, 0), path_(p, 1)) == 1) {
        starred_(path_(p, 0), path_(p, 1)) = 0;
      }
      else {
        starred_(path_(p, 0), path_(p, 1)) = 1;
      }
    }
  }

  void Munkres::erase_primes()
  {
    for(int i = 0; i < N_; i++) {
      for(int j = 0; j < N_; j++) {
        if(starred_(i, j) == 2) {
          starred_(i, j) = 0;
        }
      }
    }
  }

  int Munkres::find_epsilon() {
    int epsilon = INT_MAX;
    for(int i = 0; i < N_; i++) {
      for(int j = 0; j < N_; j++) {
        if(!row_cover_[i] && !col_cover_[j]) {
          epsilon = (epsilon > C_(i, j)) ? C_(i, j) : epsilon;
        }
      }
    }
    return epsilon;
  }
}
