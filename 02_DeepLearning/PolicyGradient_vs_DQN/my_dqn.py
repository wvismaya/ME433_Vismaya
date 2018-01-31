import myprep

#############################################
######### MAIN CODE
#############################################

env = gym.make('CartPole-v1')
state_size = env.observation_space.shape[0]
action_size = env.action_space.n

scores, episodes, statex, stateth = [], [], [], []
EPISODES = 500

savedmodel = Path("cartpole1_dqn.h5")
if savedmodel.is_file():
    # file exists
    cp_model = load_model('cartpole1_dqn.h5')
else:
    cp_model = build_model(state_size)

target_model = load_model('cartpole_pg.h5')

try:
    for e in range(EPISODES):
        done = False
        score = 0
        state = env.reset()
        state = np.reshape(state, [1, state_size])
        
        while not done:
            # Epsilon Greedy Policy
            if np.random.rand() <= epsilon:
                action = random.randrange(action_size)
            else:
                q_value = cp_model.predict(state)
                action = np.argmax(q_value[0])
            
            next_state, reward, done, info = env.step(action)
            next_state = np.reshape(next_state, [1, state_size])
            reward = reward if not done or score == 499 else -100
            
            dqn_memory.append((state, action, reward, next_state, done))
            if epsilon > epsilon_min:
                epsilon *= epsilon_decay
            # every time step do the training
            train_model_dqn(cp_model, target_model, dqn_memory)
            score += reward
            state = next_state
            
            if done:
                target_model.set_weights(cp_model.get_weights())
                score = score if score == 500 else score + 100
                scores.append(score)
                # Append as time of learning
                episodes.append(e*0.02)
                statex.append(state[0][0])
                stateth.append(state[0][2])
                
                if np.mean(scores[-min(10, len(scores)):]) > 490:
                    print("Found solution")
                    #break
                    raise SuccessReached

except SuccessReached:
    cp_model.save("cartpole1_dqn.h5")
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