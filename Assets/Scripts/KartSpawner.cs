using System;
using Cinemachine;
using Kart.MLA;
using UnityEngine;
using Utilities;
using Object = UnityEngine.Object;
using Random = UnityEngine.Random;

namespace Kart
{
    enum AIType
    {
        Regular,
        ReinforcedLearning
    }
    public class KartSpawner : MonoBehaviour
    {
        [SerializeField] private Transform[] wayPoints;
        [SerializeField] private Transform[] spawnPoints;
        [SerializeField] private Circuit circuit;
        [SerializeField] private AIDriverData aiDriverData;
        [SerializeField] private GameObject[] aiKartPrefabs;
        
        [SerializeField] private GameObject playerKartPrefab;
        [SerializeField] private CinemachineVirtualCamera playerCamera;
        [SerializeField] private AIType aiType;

        private void Awake()
        {
            if (wayPoints.Length != 0)
            {
                circuit.waypoints = wayPoints;    
            }

            if (spawnPoints.Length != 0)
            {
                circuit.spawnPoints = spawnPoints;
            }
        }

        void Start() {
            var playerKart = Instantiate(playerKartPrefab, circuit.spawnPoints[0].position, circuit.spawnPoints[0].rotation);
            playerCamera.Follow = playerKart.transform;
            playerCamera.LookAt = playerKart.transform;
            // Spawn AI Karts
            if (aiType == AIType.Regular)
            {
                for (int i = 1; i < circuit.spawnPoints.Length; i++) {
                    new AIKartBuilder(aiKartPrefabs[Random.Range(0, aiKartPrefabs.Length)])
                        .withCircuit(circuit)
                        .withDriverData(aiDriverData)
                        .withSpawnPoint(circuit.spawnPoints[i])
                        .build();
                }
            } 
            // else if (aiType == AIType.ReinforcedLearning)
            // {
            //     new AIKartBuilder(aiKartPrefabs[Random.Range(0, aiKartPrefabs.Length)])
            //         .withCircuit(circuit)
            //         .withSpawnPoint(circuit.spawnPoints[1])
            //         .buildML();
            // }

        }

        class AIKartBuilder {
            GameObject prefab;
            AIDriverData data;
            Circuit circuit;
            Transform spawnPoint;

            public AIKartBuilder(GameObject prefab) {
                this.prefab = prefab;
            }
            
            public AIKartBuilder withDriverData(AIDriverData data) {
                this.data = data;
                return this;
            }
            
            public AIKartBuilder withCircuit(Circuit circuit) {
                this.circuit = circuit;
                return this;
            }
            
            public AIKartBuilder withSpawnPoint(Transform spawnPoint) {
                this.spawnPoint = spawnPoint;
                return this;
            }

            public GameObject build() {
                var instance = Object.Instantiate(prefab, spawnPoint.position, spawnPoint.rotation);
                var aiInput = instance.GetOrAdd<AIInput>();
                aiInput.AddCircuit(circuit);
                aiInput.AddDriverData(data);
                instance.GetComponent<KartController>().SetInput(aiInput);
                
                return instance;
            }

            // public GameObject buildML()
            // {
            //     var instance = Object.Instantiate(prefab, spawnPoint.position, spawnPoint.rotation);
            //     var agent = instance.GetOrAdd<KartAgent>();
            //     agent.circuit = circuit;
            //     instance.GetComponent<KartController>().SetInput(agent);
            //     return instance;
            // } 
        }
    }
}