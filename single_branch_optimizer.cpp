#include "single_branch_optimizer.hpp"

#include <cstdlib>
#include <iostream>

#include <bits/stdc++.h>

#include <utility>

#include <libpll/pll.h>
#include <pll_partition.hpp>
#include <pll_util.hpp>

#include "../authority.hpp"
#include "../common.hpp"
#include "../options.hpp"

namespace pt { namespace move_tester {

SingleBranchOptimizer::SingleBranchOptimizer(const Options& options) :
    MoveTester(options)
{ }

SingleBranchOptimizer::~SingleBranchOptimizer()
{ }

std::pair<bool, double>
SingleBranchOptimizer::EvaluateMove(pll::Partition& partition,
                                    pll_utree_t* /* tree */,
                                    pll_unode_t* node,
                                    MoveType type,
                                    const Authority& authority) const
{
  // apply move and invalidate the CLVs on that edge
  pll_utree_nni(node, type, nullptr);
  pll::InvalidateEdgeClvs(node);

  const double original_length = node->length;

  // orient CLVs and optimize branch
  partition.TraversalUpdate(node, pll::TraversalType::PARTIAL);
  partition.OptimizeBranch(node);

  // no need for another traversal, since the CLVs are already
  // pointing at node

  double test_score;
  if (GetOptions().marginal_mode) {
    test_score = partition.LogMarginalLikelihood(node);
  } else {
    test_score = partition.LogLikelihood(node);
  }

  // we're just testing whether or not to try the move, so we don't
  // report the score to the authority yet
  bool accept_move = false;
  // if (test_score >= authority.GetThresholdScore()) {
  //   accept_move = true;
  // }


// NEW BROAD LIKELIHOOD BASED FUNCTION
  if (test_score >= authority.GetThresholdScore()) {
    if(test_score >= (authority.GetThresholdScore() )){
      accept_move = true;
    }
    else {
      srand(time(0));
      double p;
      p = (test_score - authority.GetThresholdScore());
      float t;
      t = ((float)rand()) / RAND_MAX;
      if(p<=2){
        if(t<1){
          accept_move = true;
        }
      }
      if(p<=3){
        if(t<0.5){
          accept_move = true;
        }
      }
      if(p<=4){
        if(t<0.25){
          accept_move = true;
        }
      }
      if(p<=5){
        if(t<0.125){
          accept_move = true;
        }
      }
      if(p<=6){
        if(t<0.0625){
          accept_move = true;
        }
      }
      if(p<=7){
        if(t<0.03125){
          accept_move = true;
        }
      }
      else{
        accept_move = false;
      }
    }
  }


  //MAIN LIKELIHOOD BASED FUNCTION !!!!!!
  // if (test_score >= authority.GetThresholdScore()) {
  //   if(test_score >= (authority.GetThresholdScore() + 1)){
  //     accept_move = true;
  //   }
  //   else {
  //     srand(time(0));
  //     double p;
  //     p = (test_score - authority.GetThresholdScore())/1;
  //     float t;
  //     t = ((float)rand()) / RAND_MAX;
  //     if(t <= p){
  //       accept_move = true;
  //     }
  //     else{
  //       accept_move = false;
  //     }
  //   }
  // }

  // if (test_score >= authority.GetThresholdScore()) {
  //   srand(time(0));
  //   float t;
  //   t = ((float)rand()) / RAND_MAX;
  //   if(t >= 0.5){
  //     accept_move = true;
  //   }
  //   else{
  //     accept_move = false;
  //   }
  // }


  // restore the branch length, undo the move, and invalidate the CLVs
  // on that edge again. future operations on this tree will require a
  // traversal first.
  partition.UpdateBranchLength(node, original_length);
  pll_utree_nni(node, type, nullptr);
  pll::InvalidateEdgeClvs(node);

  return std::make_pair(accept_move, test_score);
}

} } // namespace pt::move_tester
