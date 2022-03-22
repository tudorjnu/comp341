# Reinforcement Learning

Reinforcement learning allows agents to learn an unknown, complex environment through interaction with it. 

## Intro

![Reinforcement Learning loop[^1]](../images/rl_diagram.png)



The RL setup is fairly simple. We have an environment and an agent. The agent receives an observation from the environment and decides what action to take. Based on this, the agent arrives in a new state and receives a reward. We are going to work with the simplest case, the one in which we our decisions do not change the environment.



Let's assume that we move to a new town and we want to find which restaurant is the best. Each day we can eat in one restaurant only and we will assume that our satisfaction can be measured from a scale of one to ten. How can we find the best restaurant? That is, the restaurant that gives us the maximum satisfaction?

We can, for example, pick a random restaurant and see what satisfaction we get. In reinforcement learning, this is called **reward**. Now, imagine that we pick another restaurant, in the hope that it will be better than the first. Let's assume that it is not. What can we do about it? Do we explore more restaurants or do we exploit the first restaurant that we were satisfied with. This is called the **exploration-exploitation**. We will use the epsilon-greedy policy to chose what action we will take. At each timestep, with a probability of $`p`$, we will chose a random action while with a probability of $`p-1`$ we will choose the greedy action (i.e the action that brings the maximum reward).

Now let's add the reasonable assumption that there are different chiefs in the restaurants and each time the food is slightly different (i.e one time is better, one time is worse). How can we estimate the reward? 

One way is to compute the mean of the rewards and use it as a reference. One problem though, we will have to remember all of the rewards we received so far. Fortunately, we can use an incremental average:

```math
\mu_t = \mu_{t-1} + \frac{r_t - \mu_{t-1}}{t}
```

where $`\mu_t`$ is the mean at time $`t`$, $`\mu_{t-1}`$ is the mean at time $`t-1`$ and $`r_t`$ is the reward received at time $`t`$.
 

## Multi-Armed Bandits

![Multi-Armed Bandits[^2]](../images/mabd.jpeg)



A classical problem in Reinforcement Learning is called the Multi-Armed Bandits. It is very similar to our problem, in the sense that at each timestep, our agent can pull an arm of the slot machine. Each time, it randomly receives a reward based on a gausian distribution. The task is to find the arm that gives the biggest reward.

## Your Task

Design a multi armed bandit environment using the gym interface.

A sample code is provided on the github repository at `code/mabd.py`. As before, either download it manually from [here](https://github.com/tudorjnu/comp341.git) or use the following command within the `comp341` directory:

```
git pull
```

[^1]: https://towardsdatascience.com/reinforcement-learning-rl-101-with-python-e1aa0d37d43b
[^2]: https://paperswithcode.com/task/multi-armed-bandits