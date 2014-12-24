// Copyright (c) 2014, Jing Fan. All Rights Reserved.

#include <cstdlib>
#include <cstring>
#include <iostream>

#include "quickbsp/vertex_layer-inl.h"
#include "quickbsp/engine-inl.h"

struct VertexValue {
  explicit VertexValue(double s) : score(s) {}
  VertexValue() : score(0) {}  
  double score;
};


class MVertex:public graph::BaseVertex<VertexValue, void, double> {
 public:
  MVertex(const graph::node_id vertex_id) : BaseVertex(vertex_id) {}
  
  void Compute(double message_value) {
    set_acc_message_value(this->acc_message_value() + message_value);
  }

  void Reset() {
    set_acc_message_value(0);
  }

  double GetMessageValue() {
    return this->vertex_value().score/this->out_len();
  }

  void UpdateValue() {
    this->set_vertex_value(VertexValue(0.85*this->acc_message_value() + 0.15));
    this->set_acc_message_value(0);
  }
  
  graph::GatherEdgeType GetSendEdgeType() const {
    return graph::GatherEdgeType::kAllOutEdges;
  }
};

typedef graph::Edge<void> MEdgeType;



int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << "ERROR: argument length should be 3" << std::endl;
    exit(-1);
  }

  const char* filename = argv[1];
  int thread_num = atoi(argv[2]);

  typedef graph::Graph<MVertex, void> GraphType;
  GraphType* graph_ptr = (new graph::GraphLoader<GraphType>)->Load(filename);
  graph_ptr->Finalize();
  for (int i = 0 ; i < graph_ptr->num_vertices(); i ++) {
    graph_ptr->GetVertexByIndex(i).Reset();
  }

  typedef bsp::Engine<GraphType, MVertex, MEdgeType, double> Engine;
  std::unique_ptr<Engine> engine_ptr 
      = std::unique_ptr<Engine>(new Engine(thread_num,
                                           graph_ptr,
                                           bsp::PartitionType::kPartitionRoundRobin,
                                           20));
  engine_ptr->Run();
  for (int i = 0 ; i < 100 ; i ++) {
    std::cout << graph_ptr->GetVertexByIndex(graph_ptr->GetIndexByNodeId(i)).vertex_value().score << std::endl;
  }
}
