#ifndef TSPACTION_H
#define TSPACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_router/TSPAction.h>
#include <iostream>
#include <vector>

class TSPAction{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<robot_router::TSPAction> as_;
        std::string action_name_;
        robot_router::TSPFeedback feedback_;
        robot_router::TSPResult result_;
        int tamanhoSolucao;
        bool success;
        std::vector<std::vector<double>> matrizAdj;
        int dimension;
        struct tLocais {
            int distancia;
            int i;
            int localInsercao;
        };
        std::vector<int> ils_rvnd();

        int GenerateRandomNumber(int tamanho);
        void printData();
        void MelhorInsercao(std::vector<int>& solucao,
                            int escolhido,
                            std::vector<tLocais>& melhorDistancia,
                            std::vector<int>& conjuntoLocais);
        void ExcluirValorEscolhido(std::vector<int>& conjuntoDeLocais, int localInsercao);
        void InsercaoMaisBarata(std::vector<int>& conjuntoDeLocais,
                                std::vector<int>& solucao,
                                std::vector<tLocais>& melhorCaminho);
        void Limitar_Variacoes_Dos_Indices(int& indiceInicial,
                                        int& indiceFinal);

        void VerificarLimitesParaIndices(int& indiceInicial1,
                                        int& indiceFinal1,
                                        int& indiceInicial2,
                                        int& indiceFinal2);
        static bool Ordena(tLocais a, tLocais b);
        double Algoritmo_RVND(std::vector<int>& solucao,
                            double distancia,
                            int interacaoNoMomento);
        double Reinsertion(std::vector<int>& solucao,
                        double distancia,
                        int tamanho);
        double Swap(std::vector<int>& solucao,
                    double distancia);
        double Two_OPT(std::vector<int>& solucao,
                    double distancia);
        double DoubleBridge_Pertubation(std::vector<int>& solucao,
                                        double distancia);
    public:
        TSPAction(std::string name) : as_(nh_, name, boost::bind(&TSPAction::executeCB, this, _1), false), action_name_(name){
            as_.start();
        }

        ~TSPAction(void){
            
        }

        void executeCB(const robot_router::TSPGoalConstPtr &goal){
            success = true;
            auto pontos = goal->tagCoordinates;
            ROS_INFO("Instance dimension: %d", pontos.size());
            dimension = pontos.size();
            tamanhoSolucao = dimension + 1;

            matrizAdj = std::vector<std::vector<double>>(dimension,std::vector<double>());

            for (int i = 0; i < dimension; i++) {
                matrizAdj[i] = std::vector<double>(dimension);
                for (int j = 0; j < dimension; j++) {
                    matrizAdj[i][j] = abs(pontos[i].x - pontos[j].x) +
                                        abs(pontos[i].y - pontos[j].y);
                }
            }

            ROS_INFO("Optimization started!");
            result_.sequence = ils_rvnd();
            ROS_INFO("Optimization ended!");
            if(success)
                as_.setSucceeded(result_);
        }
};

#endif // !TSPACTION_H