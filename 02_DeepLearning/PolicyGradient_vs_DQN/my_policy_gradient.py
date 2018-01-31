import myprep

#############################################
######### MAIN CODE
#############################################
env = gym.make('CartPole-v1')
state_size = env.observation_space.shape[0]
action_size = env.action_space.n

savedmodel = Path("cartpole_pg.h5")
if savedmodel.is_file():
    # file exists
    cp_model = load_model('cartpole_pg.h5')
else:
    cp_model = build_model(state_size)

scores, episodes, statex, stateth = [], [], [], []
EPISODES = 500


try:
    for e in range(EPISODES):
        done = False
        score = 0
        newstate = env.reset()*np.random.uniform(low=-1.02, high=1.02)
        newstate = np.reshape(newstate, [1, state_size])
        
        states, actions, rewards = [], [], []
        
        while not done:
            policy = cp_model.predict(newstate, batch_size=1).flatten()
            action =  np.random.choice(action_size, 1, p=policy)[0]
            
            next_state, reward, done, info = env.step(action)
            next_state = np.reshape(next_state, [1, state_size])
            reward = reward if not done or score == 499 else -100
            
            states.append(newstate)
            rewards.append(reward)
            actions.append(action)
            
            score += reward
            newstate = next_state
            
            if done:
                train_model_policy(cp_model, states, actions, rewards)
                states, actions, rewards = [], [], []
                
                score = score if score == 500 else score + 100
                scores.append(score)
                episodes.append(e*0.02)
                statex.append(newstate[0][0])
                stateth.append(newstate[0][2])
                # Check last 'n' mean score
                if np.mean(scores[-min(10, len(scores)):]) > 490:
                    print('Found Solution')
                    raise SuccessReached
except SuccessReached:
    cp_model.save("cartpole_pg.h5")
    pass

plt.figure(1)
plt.subplot(211)
plt.plot(episodes, statex)
plt.plot(episodes, stateth)
plt.xlabel('Time')
plt.ylabel('Magnitude (m / rad)')
plt.legend(['X', 'Theta'])
plt.title('States over Time')

plt.subplot(212)
plt.plot(episodes, scores)
plt.xlabel('Episode time')
plt.ylabel('Scores')
plt.legend(['Scores'])
plt.title('Score over Episodes')

plt.show()

######################################
#### REFERENCES
######################################
## Woongwon, Youngmoo, Hyeokreal, Uiryeong, Keon, "Minimal and Clean Reinforcement Learning Examples", (2017 commit), Github Repository , https://github.com/rlcode/reinforcement-learning
## Giuseppe Bonaccorso, "cartpole.py" (2016), Github Code , https://gist.github.com/giuseppebonaccorso/7040b10a13520c4b0340b8a89dc8262f
## Henry Jia, "OpenAI gym cartpole" (2016), Github Repository, https://github.com/HenryJia/cartpole
## Abhishek Mishra, "pg_rnn" (2017), Github Repository, https://github.com/abhishm/pg_rnn 
######################################