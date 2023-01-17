#include "logica_node.h"

void cleanVariables(){
    edges.clear();
    blocks_list.clear();
    tokens.clear();
    for (int i = 0; i < num_block; i++)
        delete[] graph.head[i];
    delete[] graph.head;
    num_block = num_rows = num_col = 0;
    graph.N=0;
}

void give_InitToken()
{
    // create vector with all vertices
    for (int i = 0; i < num_block; i++)
    {
        tokens.push_back(blocks_list[i].id);
    }

    // remove vertices that arent initials
    for (int i = 0; i < num_block; i++) 
    {
        adjNode* ptr = graph.head[i];

        while (ptr != nullptr) {
            tokens.erase(std::remove(tokens.begin(), tokens.end(), ptr->val), tokens.end());
            ptr = ptr->next;
        }
    }
    
    // remove duplicate values from vector
    auto end = tokens.end();
    for (auto it = tokens.begin(); it != end; ++it) {
        end = std::remove(it + 1, end, *it);
    }
    tokens.erase(end, tokens.end());
}

int check_InitialBlocks(std::vector <int> tokens, std::vector <blocks> blocks)
{
    for (int i = 0; i < tokens.size(); i++)
    {
        for (int j = 0; j < blocks.size(); j++)
        {
            if (tokens[i] == blocks[j].id)
            {
                if ((blocks[j].type == "operator") || (blocks[j].type == "choice") || (blocks[j].type == "comparator") ||
                    (blocks[j].type == "timer") || (blocks[j].type == "transport") || (blocks[j].type == "extend") ||
                    (blocks[j].name == "comma"))
                {
                    return -4;
                }
            }
        }
    }

    return 0;
}

int check_EdgesStupid(std::vector< std::vector<int>> matrix, int n)
{
    // procurar ligacoes confusas 
    for (int i = 0; i < n; i++)
    {
        if ((matrix[i][0] == -1) && (matrix[i][1] == -1)) // inputs todos a -1
        {
            return -3;
        }
        else if ((matrix[i][2] == -1) && (matrix[i][3] == -1) && (matrix[i][4] == -1)) // outputs todos a -1
        {
            return -3;
        }
    }

    return 0;
}

int check_EdgesQuantity(std::vector <graphEdge> edges, std::vector <blocks> blocks)
{
    // verificar se todos os i/o tem conexoes, erros por defeito ou excesso
    for (int i = 0; i < blocks.size(); i++)
    {
        int n_i1, n_i2, n_cond;
        n_i1 = n_i2 = n_cond = 0;
    
        for (int j = 0; j < edges.size(); j++)
        {
            if (edges[j].end_ver == blocks[i].id) //verificar inputs
            {
                if (edges[j].input_id == 1)
                    n_i1++;
                else if (edges[j].input_id == 2)
                    n_i2++;
                else
                    n_cond++;
            }
        }

        // 0i - 1o: sensores
        if (blocks[i].type == "sensor")
        {
            if (((n_i1 + n_i2 + n_cond) > 0))
                return -2;
        }
        // 1i - 1o: constants
        else if ((blocks[i].type == "constant") || (blocks[i].type == "motor") || (blocks[i].name == "simetric"))
        {
            if ((n_i2 + n_cond) > 0)
                return -2;
        }
        // 1i - 1o: simetrico, motores
        else if ((blocks[i].type == "motor") || (blocks[i].name == "simetric"))
        {
            if ((n_i1 != 1) || ((n_i2 + n_cond) > 0))
                return -2;
        }
        // 2i - 1o: operators (sum & product), comparators, timer, and, or
        else if ((blocks[i].name == "sum") || (blocks[i].name == "product") || (blocks[i].type == "comparator") ||
            (blocks[i].name == "delay") || (blocks[i].name == "and") || (blocks[i].name == "or"))
        {
            if ((n_i1 != 1) || (n_i2 != 1) || (n_cond > 0))
                return -2;
        }
        // 2i (cond) - 1o: if
        else if (blocks[i].name == "if")
        {
            if ((n_i1 != 1) || (n_i2 != 0) || (n_cond != 1))
                return -2;
        }
        // 3i (cond) - 1o: mux e else if
        else if ((blocks[i].name == "mux") || (blocks[i].name == "else_if"))
        {
            if ((n_i1 != 1) || (n_i2 != 1) || (n_cond != 1))
                return -2;
        }
        // 2i - 0o: TE
        else if (blocks[i].name == "t_left")
        {
            if ((n_i1 + n_i2 == 0) || (n_cond != 0))
                return -2;
        }
        // 0i - 2o: TD
        else if (blocks[i].name == "t_right")
        {
            if ((n_i1 + n_i2 + n_cond != 0))
                return -2;
        }
        // 1i - 2o: Extensor
        else if (blocks[i].name == "double")
        {
            if ((n_i1 != 1) || ((n_i2 + n_cond) != 0))
                return -2;
        }
    }
    
    return 0;
}

int check_EdgesLogic() // criar func para verificar se todas as ligacoes fazem sentido 
{
    for (int i = 0; i < num_rows; i++) 
    {
        std::string start_type = "";
        std::string end_type = "";
        std::string start_name = "";
        std::string end_name = "";

        for (int j = 0; j < blocks_list.size(); j++)
        {
            if (blocks_list[j].id == edges[i].start_ver)
            {
                start_type = blocks_list[j].type;
                start_name = blocks_list[j].name;
            }
            if (blocks_list[j].id == edges[i].end_ver)
            {
                end_type = blocks_list[j].type;
                end_name = blocks_list[j].name;
            }
        }

        if (start_type == "operator")
        {
            if ((end_type == "choice") && (edges[i].input_id == 3))
            {
                return -1;
            }
            else if (end_type == "choice")
            {
                if ((end_name == "or") || (end_name == "and"))
                {
                    return -1;
                }
            }
            else if ((end_type == "timer") && (edges[i].input_id == 1)) 
            {
                return -1;
            }
        }

        if (start_type == "choice")
        {
            if ((start_name == "or") || (start_name == "and"))
            {
                if (end_type == "operator")
                {
                    if ((end_name == "sum") || (end_name == "product"))
                    {
                        return -1;
                    }
                }
                else if (end_type == "choice")
                {
                    if ((edges[i].input_id == 1) || (edges[i].input_id == 2))
                    {
                        return -1;
                    }
                }
                else if ((end_type == "comparator") || (end_type == "motor") || 
                    (end_type == "constant") || (end_type == "timer"))
                {
                    return -1;
                }
            }
            else //mux, if, else if - float
            {
                if (end_type == "choice")
                {
                    if (edges[i].input_id == 3)
                    {
                        return -1;
                    }
                    else if ((end_name == "or") || (end_name == "and"))
                    {
                        return -1;
                    }
                }
                else if ((end_type == "timer") && (edges[i].input_id == 1))
                {
                    return -1;
                }
            }
        }

        if (start_type == "comparator")
        {
            if ((end_type == "operator") && (end_name != "simetric"))
            {
                return -1;
            }
            else if ((end_type == "choice") && (edges[i].input_id != 3))
            {
                return -1;
            }
            else if ((end_type == "comparator") || (end_type == "motor") || 
                (end_type == "constant") || (end_type == "timer"))
            {
                return -1;
            }
        }

        if ((start_type == "motor") || (start_type == "sensor") || 
            (start_type == "constant"))
        {
            if (end_type == "choice")
            {
                if ((end_name == "or") || (end_name == "and"))
                {
                    return -1;
                }
                else if (edges[i].input_id == 3)
                {
                    return -1;
                }
            }
            else if (end_type == "comparator")
            {
                return -1;
            }
            
        }

        if ((start_type == "sensor") || (start_type == "constant"))
        {
            if ((end_type == "timer") && (edges[i].input_id == 1))
            {
                return -1;
            }
        }
     
        if (start_type == "timer")
        {
            if (end_type == "choice")
            {
                if ((end_name == "or") || (end_name == "and"))
                {
                    return -1;
                }
                else if (edges[i].input_id == 3)
                {
                    return -1;
                }
            }
            else if (end_type == "timer")
            {
                return -1;
            }
        }    
    }

    return 0;
}

adjNode* getAdjListNode(int value, adjNode* head) {
    adjNode* newNode = new adjNode;
    newNode->val = value;
    newNode->next = head;   // poinh{} new node to current head
    return newNode;
}

void setSize(){
    graph.head = new adjNode * [num_block]();
    graph.N = num_block;

    // initialize head pointer for all vertices
    for (int i = 0; i < num_block; ++i)
        graph.head[i] = nullptr;

    // construct directed graph by adding edges to it
    for (int i = 0; i < num_rows; i++) {
        int start_ver = edges[i].start_ver;
        int end_ver = edges[i].end_ver;

        // insert in the beginning
        adjNode* newNode = getAdjListNode(end_ver, graph.head[start_ver]);

        // point head pointer to new node
        graph.head[start_ver] = newNode;
    }
}

void matrix_ToEdges(std::vector< std::vector<int>> matrix)
{    
    // matrix to edges
    for (int i = 0; i < num_rows; i++)
    {
        int start, end, inp, out;
        start = end = inp = out = 0;

        if (matrix[i][0] != -1)
        {
            out = 1;
            start = matrix[i][0];
        }
        else
        {
            out = 2;
            start = matrix[i][1];
        }

        if (matrix[i][2] != -1)
        {
            inp = 1;
            end = matrix[i][2];
        }
        else if (matrix[i][3] != -1)
        {
            inp = 2;
            end = matrix[i][3];
        }
        else
        {
            inp = 3;
            end = matrix[i][4];
        }

        graphEdge gE = { start, end, out, inp };
        edges.push_back(gE);
    }
} 

void construct_Blocks(std::vector <int> id, int N) // inicializa vetor com todos os blocos
{
    for (int i = 0; i < N; i++)
    {
        blocks newBlock;
        newBlock.id = i;
        newBlock.class_id = id[i];
        newBlock.out_f = NAN;
        newBlock.out_b = NAN;
        newBlock.constant = "";
        newBlock.timer = NULL;
        newBlock.time_done = false;

        switch (id[i])
        {
            case 0:
                newBlock.type = "operator";
                newBlock.name = "sum";
                break;
            case 1:
                newBlock.type = "operator";
                newBlock.name = "product";
                break;
            case 2:
                newBlock.type = "operator";
                newBlock.name = "simetric";
                break;
            case 27:
                newBlock.type = "choice";
                newBlock.name = "mux";
                break;
            case 3:
                newBlock.type = "choice";
                newBlock.name = "if";
                break;
            case 32:
                newBlock.type = "choice";
                newBlock.name = "else_if";
                break;
            case 25:
                newBlock.type = "choice";
                newBlock.name = "and";
                break;
            case 26:
                newBlock.type = "choice";
                newBlock.name = "or";
                break;
            case 4:
                newBlock.type = "comparator";
                newBlock.name = "less";
                break;
            case 5:
                newBlock.type = "comparator";
                newBlock.name = "greater";
                break;
            case 6:
                newBlock.type = "comparator";
                newBlock.name = "equal";
                break;
            case 7:
                newBlock.type = "motor";
                newBlock.name = "m_left";
                break;
            case 8:
                newBlock.type = "motor";
                newBlock.name = "m_right";
                break;
            case 9:
                newBlock.type = "sensor";
                newBlock.name = "s_left";
                break;
            case 11:
                newBlock.type = "sensor";
                newBlock.name = "s_front";
                break;
            case 10:
                newBlock.type = "sensor";
                newBlock.name = "s_right";
                break;
            case 12:
                newBlock.type = "sensor";
                newBlock.name = "s_back";
                break;
            case 14:
                newBlock.type = "constant";
                newBlock.name = "zero";
                break;
            case 15:
                newBlock.type = "constant";
                newBlock.name = "one";
                break;
            case 16:
                newBlock.type = "constant";
                newBlock.name = "two";
                break;
            case 17:
                newBlock.type = "constant";
                newBlock.name = "three";
                break;
            case 18:
                newBlock.type = "constant";
                newBlock.name = "four";
                break;
            case 19:
                newBlock.type = "constant";
                newBlock.name = "five";
                break;
            case 20:
                newBlock.type = "constant";
                newBlock.name = "six";
                break;
            case 21:
                newBlock.type = "constant";
                newBlock.name = "seven";
                break;
            case 22:
                newBlock.type = "constant";
                newBlock.name = "eight";
                break;
            case 23:
                newBlock.type = "constant";
                newBlock.name = "nine";
                break;
            case 24:
                newBlock.type = "constant";
                newBlock.name = "comma";
                break;
            case 13:
                newBlock.type = "timer";
                newBlock.name = "delay";
                break;
            case 34:
                newBlock.type = "transport";
                newBlock.name = "t_left";
                break;
            case 35:
                newBlock.type = "transport";
                newBlock.name = "t_right";
                break;
            case 33:
                newBlock.type = "extend";
                newBlock.name = "duplicate";
                break;
            default:
                break;
        }

        blocks_list.push_back(newBlock);
    }
}

void subs_Extend()
{
    bool rem_blocks = false;
    int count = 0;

    for (int i = 0; i < num_block; i++) // search block extend
    {   
        if (blocks_list[i].name == "duplicate")
        {   
            for (int k = 0; k < num_rows; k++) // search edges
            {   
                if (edges[k].end_ver == blocks_list[i].id)
                {   
                    for (int l = 0; l < num_rows; l++)
                    {   
                        if (edges[l].start_ver == blocks_list[i].id)
                        {   
                            graphEdge edge;
                            edge = { edges[k].start_ver, edges[l].end_ver, edges[k].output_id, edges[l].input_id };

                            // remove and add 1st edge
                            if (count == 0)
                            {
                                edges.erase(edges.begin() + l);
                                edges.push_back(edge);
                                k = l = 0;
                                count++;
                            }
                            else // remove 2 edges and add 2nd edge
                            {
                                edges.erase(edges.begin() + k);
                                if (l < k)
                                {
                                    edges.erase(edges.begin() + l);
                                }
                                else
                                {
                                    edges.erase(edges.begin() + l - 1);
                                }
                                edges.push_back(edge);
                                count = 0;
                                rem_blocks = true;
                                num_rows--;
                            }
                        }
                    }
                }
            }

            

            //remove block
            if (rem_blocks)
            {
                blocks_list.erase(blocks_list.begin() + i);
                num_block--;
                rem_blocks = false;
            }

            
        }
    }
}

void cbMatrix(const serp::Matrix::ConstPtr &msg){
    ROS_INFO("CALLBACK");
   
    if(msg->manual_mode){
        out_vel[0] = msg->vel_motor_left;
        out_vel[1] = msg->vel_motor_right;
        matrix_rcv = false;
    }
    else{
        matrix_rcv = true; 
        num_block = msg->matrix1.size();
        num_rows = msg->matrix2.size() / 5;
        num_col = 5;
        std::vector<int> matrix1;
        for(int i; i < num_block; i++){
            matrix1.push_back(msg->matrix1[i]);
        }

        std::vector<std::vector<int>> matrix2;
        matrix2.resize(num_rows);
        for(int i = 0; i < num_rows; i++ ){
            matrix2[i].resize(num_col);
        }

        for (int i = 0; i < num_rows*num_col; i++)
        {
            matrix2[i / 5][i % 5] = msg->matrix2[i];
        }

        // print matrix
        // std::cout << "Matrix 1" << std::endl;
        // for(int i = 0; i < matrix1.size(); i++ ){
        //     std::cout << matrix1.at(i) << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "Matrix 2" << std::endl;
        // for (int i = 0; i < matrix2.size(); i++)
        // {
        //     for (int j = 0; j < matrix2[i].size(); j++)
        //     {
        //         std::cout << matrix2[i][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }

        construct_Blocks(matrix1, num_block);
        matrix_ToEdges(matrix2);
        subs_Extend();
        setSize();
        give_InitToken();

        error[0] = check_EdgesLogic();
        error[1] = check_EdgesQuantity(edges, blocks_list);
        error[2] = check_EdgesStupid(matrix2, num_rows);
        error[3] = check_InitialBlocks(tokens, blocks_list);

        for(int i=0; i < 4; i++){
            if(error[i] < 0){
                std_msgs::Int8 e;
                e.data = error[i];
                pub_errors.publish(e);
                return;
            }
        }

        std::cout << "ACABOU" << graph.N << std::endl;
    }
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "logic_node");
  
  ros::NodeHandle node;
  
  ros::Subscriber matrix_sub = node.subscribe("/matrix", 1, cbMatrix);
  //ros::Subscriber sensors_sub = node.subscribe("/sensors", 1, cbSensors);

  pub_vel = node.advertise<serp::Velocity>("/vel", 1);
  out_vel[0] = 0;
  out_vel[1] = 0;

  serp::Velocity vel;

  pub_errors = node.advertise<std_msgs::Int8>("/error_logic", 1);

  while(ros::ok()){
    if(matrix_rcv){
       //std::cout << "HERE" << std::endl;
    }
    
    vel.vel_motor_left = out_vel[0];
    vel.vel_motor_right = out_vel[1];
    pub_vel.publish(vel);
    ros::spinOnce();
  }

  return 0;
}