#include "branch_neighborhood_optimizer.hpp"

#include <utility>

#include <bits/stdc++.h>

#include <libpll/pll.h>
#include <pll_partition.hpp>
#include <pll_util.hpp>

#include "../authority.hpp"
#include "../common.hpp"
#include "../options.hpp"

namespace pt { namespace move_tester {

BranchNeighborhoodOptimizer::BranchNeighborhoodOptimizer(const Options& options) :
    MoveTester(options)
{
  if (GetOptions().optimization_radius <= 0) {
    throw std::invalid_argument("optimization radius must be positive");
  }
}

BranchNeighborhoodOptimizer::~BranchNeighborhoodOptimizer()
{ }

std::pair<bool, double>
BranchNeighborhoodOptimizer::EvaluateMove(pll::Partition& partition,
                                          pll_utree_t* original_tree,
                                          pll_unode_t* original_node,
                                          MoveType type,
                                          const Authority& authority) const
{
  //
  // since we're optimizing more than one branch it makes sense to
  // operate on a clone of the given tree rather than try and keep
  // track of all the branch lengths we need to restore. the downside
  // is that the partition can get out of sync with the original tree.
  //

  // clone the underlying tree structure and wrap it up so we have a
  // reference to the right node but don't have to change the tree we
  // were passed
  pll_unode_t* node = pll_utree_graph_clone(original_node);
  pll_utree_t* tree = pll_utree_wraptree(node, original_tree->tip_count);
  pll_utree_every(tree, pll::cb_copy_clv_traversal);

  // apply move and invalidate the CLVs on that edge
  pll_utree_nni(node, type, nullptr);
  pll::InvalidateEdgeClvs(node);

  // orient CLVs and optimize branch neighborhood
  partition.TraversalUpdate(node, pll::TraversalType::PARTIAL);
  partition.OptimizeBranchNeighborhood(node, GetOptions().optimization_radius);

  // reorient CLVs and compute log-likelihood
  partition.TraversalUpdate(node, pll::TraversalType::PARTIAL);

  double test_score;
  if (GetOptions().marginal_mode) {
    test_score = partition.LogMarginalLikelihood(node);
  } else {
    test_score = partition.LogLikelihood(node);
  }

  // we're just testing whether or not to try the move, so we don't
  // report the score to the authority yet

  bool accept_move = false;

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



  // This is the implementation of another randomised method based on the mod function
  // if (test_score >= authority.GetThresholdScore()) {
  //   accept_move = true;
  // }

  // if (test_score >= authority.GetThresholdScore()) {
  //   if((rand()%2) == 0){
  //     accept_move = true;
  //   }
  //   else{
  //     accept_move = false;
  //   }
  // }

  
  // // THIS IS THE FIRST LIKELIHOOD BASED FUNCTION
  // if (test_score >= authority.GetThresholdScore()) {
  //   if(test_score >= (authority.GetThresholdScore() + 1)){
  //     accept_move = true;
  //   }
  //   else {
  //     srand(time(0));
  //     float p;
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

  //// This is the implementation of the randomised 50% probability selection method
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




  // we're done with the cloned tree
  pll_utree_destroy(tree, pll::cb_erase_data);

  // we're the only ones who know that we desynchronized the partition
  // and the original tree, so perform a full traversal to fix things
  partition.TraversalUpdate(original_node, pll::TraversalType::FULL);

  return std::make_pair(accept_move, test_score);
}

} } // namespace pt::move_tester
