============
Introduction
============

.. contents:: Table of Contents

What This Is
============

Welcome to Spinning Up in Deep RL for mobile robotics! This is an educational resource inspired by the Openai spinning up in Deep RL

For the unfamiliar: `reinforcement learning`_ (RL) is a machine learning approach for teaching agents how to solve tasks by trial and error. Deep RL refers to the combination of RL with `deep learning`_.

This module contains a variety of helpful resources, including:

- a short `introduction`_ to RL terminology, kinds of algorithms, and basic theory,
- an `essay`_ about how to grow into an RL research role,
- a `curated list`_ of important papers organized by topic,
- a well-documented `code repo`_ of short, standalone implementations of key algorithms,
- and a few `exercises`_ to serve as warm-ups.


.. _`reinforcement learning`: https://en.wikipedia.org/wiki/Reinforcement_learning
.. _`deep learning`: http://ufldl.stanford.edu/tutorial/

Why learn Mobile Robot navigation
=================

Mobile robot navigation is one of the oldest problems in the history of robotics. Conventional approaches in robotics focus on developing individual modules for perception, localization and control.

*Localization* - Knowing the robot’s current position in the environment ,similar to the way we enter our current location in google maps

*Mapping* - Creating a map of the robot’s environment ,similar to the way we make a mental map of a route once we drive through it.

*Control - How to move from one location to another ,similar to how we use the steering ,throttle etc to reach the destination

The overlapping of these fields are hot research topics as well.

*SLAM* - Where to look next to find out current position

*Exploration* - How to move in an environment so as to obtain the required coverage for mapping ,like the mars rover which tries to generate as large and detailed map as possible

*Active Localization* - Where to look next to find out current position

Models which implement the above modules individualy have been well studied.
However, researchers haven't been able to develop models which combine Localization, Mapping and
Control until recently. Advances in the field of deep learning, especially deep reinforcement
learning has made end-to-end trainable models possible.

Well, that may be but probabilistic robotics is quite popular and have been known to work well.
What significant improvements can end-to-end trainable models offer over proven conventional methods?

- *Learning from experience :* Conventional models cannot learn from failures or any past experiecne for that matter. Additionally, merging individual modules to produce complete robot navigation systems requires years of domain experience, extensive modelling and considerable programming effort. Prof. Wolfram Burgard, one of the autjods of the bestselling book probabilistic robotics and a key researcher in the field compared probabilistic approaches to carrying a heavy boulder up a hill thinking it would solve the problem but finally when he approached the top, he realized the that it wasn't going to make it. Most importantly, human being is not accquired as programmed modules. Rather, it we learn through numerous interactions with environments to acquire skills which can be transffered to future tasks as well.


- an `essay`_ about how to grow into an RL research role,
- a `curated list`_ of important papers organized by topic,2



Why it matters
=================

Let us consider the case of a student learning deep RL. He finds excellent course content available
from such as CS294 from Berkley by Sergey Levine and other resources such as spinning up in
Deep RL by openai offering great insights. In addition, open source environments such as the Openai
gym are available for
evaluation of state the art algorithms in simulation on a variety of standard environments.
The intuitive interface makes it easy to experiment with state of the art algorithms and compare
them with standardbenchmarks. Having exhausted all course content available online, the student
wants to do research in deep RL for mobile robot navigation because to enables end-to-end trainable
models which integrate perception,planning and control. Conventional methods use separate modules
for perception, planning and control and merging them successfully requires years of domain
experience. However, he could not findany quality course content or insightful blog posts on the
subject. The optimistic student expects thatthe same state of the algorithms which had worked in
simulation for simple control tasks but unfor-tunately, no standard environments are available
like the Openai gym.  The few resources availablerequired domain expertise and are difficult to
customize. Moreover, few published papers/literatureon the subject have open sourced the code and
discussed training setups in detail.  Another equallyimportant source of concern is that real world
 evaluations are performed on state of the art mobilerobot platforms costing thousands of dollars.
 This paper addresses the above points, which we inferhas severely limited widespread understanding
 of the limits and potentials of reinforcement learningin mobile robotics.



How This Serves Our Mission
===========================

OpenAI's mission_ is to ensure the safe development of AGI and the broad distribution of benefits from AI more generally. Teaching tools like Spinning Up help us make progress on both of these objectives.

To begin with, we move closer to broad distribution of benefits any time we help people understand what AI is and how it works. This empowers people to think critically about the many issues we anticipate will arise as AI becomes more sophisticated and important in our lives.

Also, critically, `we need people to help <https://jobs.lever.co/openai>`_ us work on making sure that AGI is safe. This requires a skill set which is currently in short supply because of how new the field is. We know that many people are interested in helping us, but don't know how---here is what you should study! If you can become an expert on this material, you can make a difference on AI safety.



Code Design Philosophy
======================

The algorithm implementations in the Spinning Up repo are designed to be

    - as simple as possible while still being reasonably good,
    - and highly-consistent with each other to expose fundamental similarities between algorithms.

They are almost completely self-contained, with virtually no common code shared between them (except for logging, saving, loading, and `MPI <https://en.wikipedia.org/wiki/Message_Passing_Interface>`_ utilities), so that an interested person can study each algorithm separately without having to dig through an endless chain of dependencies to see how something is done. The implementations are patterned so that they come as close to pseudocode as possible, to minimize the gap between theory and code.

Importantly, they're all structured similarly, so if you clearly understand one, jumping into the next is painless.

We tried to minimize the number of tricks used in each algorithm's implementation, and minimize the differences between otherwise-similar algorithms. To give some examples of removed tricks: we omit regularization_ terms present in the original Soft-Actor Critic code, as well as `observation normalization`_ from all algorithms. For an example of where we've removed differences between algorithms: our implementations of DDPG, TD3, and SAC all follow a convention laid out in the `original TD3 code`_, where all gradient descent updates are performed at the ends of episodes (instead of happening all throughout the episode).

All algorithms are "reasonably good" in the sense that they achieve roughly the intended performance, but don't necessarily match the best reported results in the literature on every task. Consequently, be careful if using any of these implementations for scientific benchmarking comparisons. Details on each implementation's specific performance level can be found on our `benchmarks`_ page.


Support Plan
============

We plan to support Spinning Up to ensure that it serves as a helpful resource for learning about deep reinforcement learning. The exact nature of long-term (multi-year) support for Spinning Up is yet to be determined, but in the short run, we commit to:

- High-bandwidth support for the first three weeks after release (Nov 8, 2018 to Nov 29, 2018).

    + We'll move quickly on bug-fixes, question-answering, and modifications to the docs to clear up ambiguities.
    + We'll work hard to streamline the user experience, in order to make it as easy as possible to self-study with Spinning Up.

- Approximately six months after release (in April 2019), we'll do a serious review of the state of the package based on feedback we receive from the community, and announce any plans for future modification, including a long-term roadmap.

Additionally, as discussed in the blog post, we are using Spinning Up in the curriculum for our upcoming cohorts of Scholars_ and Fellows_. Any changes and updates we make for their benefit will immediately become public as well.


.. _`introduction`: ../spinningup/rl_intro.html
.. _`essay`: ../spinningup/spinningup.html
.. _`Spinning Up essay`: ../spinningup/spinningup.html
.. _`curated list`: ../spinningup/keypapers.html
.. _`code repo`: https://github.com/openai/spinningup
.. _`exercises`: ../spinningup/exercises.html
.. _`rllab`: https://github.com/rll/rllab
.. _`Baselines`: https://github.com/openai/baselines
.. _`rllib`: https://github.com/ray-project/ray/tree/master/python/ray/rllib
.. _`mission`: https://blog.openai.com/openai-charter/
.. _`regularization`: https://github.com/haarnoja/sac/blob/108a4229be6f040360fcca983113df9c4ac23a6a/sac/distributions/normal.py#L69
.. _`observation normalization`: https://github.com/openai/baselines/blob/28aca637d0f13f4415cc5ebb778144154cff3110/baselines/run.py#L131
.. _`original TD3 code`: https://github.com/sfujim/TD3/blob/25dfc0a6562c54ae5575fad5b8f08bc9d5c4e26c/main.py#L89
.. _`benchmarks`: ../spinningup/bench.html
.. _Scholars : https://jobs.lever.co/openai/cf6de4ed-4afd-4ace-9273-8842c003c842
.. _Fellows : https://jobs.lever.co/openai/c9ba3f64-2419-4ff9-b81d-0526ae059f57
