#include "human_decision_maker_deformable_alg.h"

HumanDecisionMakerDeformableAlgorithm::HumanDecisionMakerDeformableAlgorithm
    (void)
{
	pthread_mutex_init(&this->access_, NULL);
}

HumanDecisionMakerDeformableAlgorithm::~HumanDecisionMakerDeformableAlgorithm
    (void)
{
	pthread_mutex_destroy(&this->access_);
}

void HumanDecisionMakerDeformableAlgorithm::config_update(Config & new_cfg,
							  uint32_t level)
{
	this->lock();

	// save the current configuration
	this->config_ = new_cfg;

	this->unlock();
}

// HumanDecisionMakerDeformableAlgorithm Public API
