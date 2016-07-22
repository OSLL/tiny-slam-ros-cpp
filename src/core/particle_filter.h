/**
 * \file
 * \brief Discribes some classes related to Particle
 * There are classes Particle, ParticleFactory, UniformResamling, ParticleFilter
 */

#ifndef __PARTICLE_FILTER_H
#define __PARTICLE_FILTER_H

#include <memory>
#include <vector>
#include <random>
#include <unordered_set>

/**
* \brief An element of ParticleFilter. Used to approximate target distribution
*/
class Particle {
public:
  Particle(): _weight(0) {}
  ///Getter function
  double weight() const { return _weight; }
  /// Setter function
  void set_weight(double w) { _weight = w; }

  virtual void sample() = 0;
private:
  double _weight; ///< Weight of a particle
};

template <typename ParticleT>
/**
 * \brief Factory for creating ParticleT (template)
 */
class ParticleFactory {
public:
  virtual std::shared_ptr<ParticleT> create_particle() = 0;
};

template <typename ParticlePtr>
class UniformResamling { /* Strategy */
public:
  // TODO: should be moved to separate strategy
  /**
   * Calculates whether resampling is requied. It's requied if \f$ \frac{2}{\sum_{i=1}^{n} w_{i}^{2}}<n \f$
   * \param particles Vector of pointers on Particle
   * \return Result of comparation
   */
  bool resampling_is_required(std::vector<ParticlePtr> particles) {
    // TODO: resampling is not required if robot haven't traveled far enough
    // Computer N_eff from gMapping (based on Doucet work)
    double sq_sum = 0;
    for (auto &p : particles) {
      sq_sum += p->weight() * p->weight();
    }
    uint effective_particles_cnt = 1.0 / sq_sum;
    return effective_particles_cnt * 2 < particles.size();
  }
  /**
   * Excludes some random particles from vector.
   * Choosing is based on generation of random double and its comparation with sum of all weights of particles
   * \param particles Input vector of pointers on Particle
   * \return Vector of indexes of some chosen points
   */
  std::vector<unsigned> resample(std::vector<ParticlePtr> particles) {
    // TODO: implement O(n) resampling from gmapping
    //double total_weight = 0;
    //for (auto &p : particles) { total_weight += particles.weight; }
    std::vector<unsigned> new_particle_inds(particles.size());

    std::random_device rd;
    std::mt19937 engine(rd());
    std::uniform_real_distribution<> uniform_distr(0, 1);
    for (uint i = 0; i < particles.size(); i++) {
      double sample = uniform_distr(engine);
      double total_w = 0;
      for (uint j = 0; j < particles.size(); j++) {
        total_w += particles[j]->weight();
        if (sample < total_w) {
          new_particle_inds[i] = j;
          break;
        }
      }
    }
    return new_particle_inds;
  }
};

/**
 * \brief Class managing vector of Particles and Resumpler
 */
template <typename ParticleT>
class ParticleFilter {
private:
  using ParticlePtr = std::shared_ptr<ParticleT>;
public: // methods
  /**
   * Constructor with parameters. Sets as default weight of each particle equal to \f$ \frac{1}{n} \f$
   * \param p_ftry Pointer on Particle Factory
   * \param n Amount of particles
   */
  ParticleFilter(std::shared_ptr<ParticleFactory<ParticleT>> p_ftry,
                 unsigned n = 1) : _particle_supplier{p_ftry} {
    for (unsigned i = 0; i < n; i++) {
      ParticlePtr particle = p_ftry->create_particle();
      particle->set_weight(1.0 / n);
      _particles.push_back(particle);
    }
  }

  /**
   * Realizes whether resample is requied and makes a resampling if it's necessary
   * \return True if resampling has happened and False otherwise
   */
  bool try_resample() {
    // TODO: heuristic with traveled distange, move to common sample_particles
    if (!_resampler.resampling_is_required(_particles)) {
      return false;
    }

    std::vector<ParticlePtr> new_particles;
    std::unordered_set<unsigned> new_part_inds;
    for (unsigned i : _resampler.resample(_particles)) {
      ParticlePtr sampled = _particles[i];
      if (new_part_inds.count(i)) {
        ParticlePtr new_particle = _particle_supplier->create_particle();
        *new_particle = *sampled;
        new_particle->sample();
        sampled = new_particle;
      } else {
        new_part_inds.insert(i);
      }
      new_particles.push_back(sampled);
    }
    _particles = std::move(new_particles);
    normalize_weights();
    return true;
  }

  /**
   * Sets weights of all particles to be equal to their average
   */
  void normalize_weights() {
    double total_weight = 0;
    for (auto &p : _particles) { total_weight += p->weight(); }
    for (auto &p : _particles) { p->set_weight(p->weight() / total_weight); }
  }

  ///Getter of link on particles
  inline std::vector<ParticlePtr>& particles() { return _particles; }
  ///Getter of constant link on particles
  inline const std::vector<ParticlePtr>& particles() const {
    return _particles;
  }
private:
  std::shared_ptr<ParticleFactory<ParticleT>> _particle_supplier; ///< Shared pointer on Particle factory
  std::vector<ParticlePtr> _particles; ///< Vector of particles
  /* TODO: make template parameter/set at runtime */
  UniformResamling<ParticlePtr> _resampler; ///< Object of Resampler class
};


#endif // header-guard
