use std::collections::{BinaryHeap, VecDeque};
use std::fs::File;
use std::io::{BufRead, BufReader};

fn main() -> Result<(), Box<dyn std::error::Error>>  {
    let maze_map_path = "data/maze_map.txt";

    println!("target_maze_data = {}", maze_map_path);

    let maze = read_maze_from_file(maze_map_path)?;
    let start_pos = find_position(&maze, &'S').unwrap();
    let goal_pos = find_position(&maze, &'G').unwrap();
    let h = maze.len() as i32;
    let w = maze[0].len() as i32;
    
    // スタート地点とゴール地点の確認
    println!(
        "size ({}, {}), start_pos {:?}, goal_pos {:?}", 
        h, 
        w, 
        start_pos, 
        goal_pos
    );

    // bfsによる探索
    let result = normal_bfs(&maze, h, w, start_pos, goal_pos);
    let cost = result.0;
    let search_tile_num = result.1;

    println!("------------ method: BFS -------------------");
    println!("start to goal path cost = {}", cost);
    println!("search tile num = {}", search_tile_num);

    // A* BFSによる探索
    let result = a_star_bfs(&maze, h, w, start_pos, goal_pos);
    let cost = result.0;
    let search_tile_num = result.1;

    println!("------------ method: BFS -------------------");
    println!("start to goal path cost = {}", cost);
    println!("search tile num = {}", search_tile_num);
    Ok(())
}

fn normal_bfs(maze: &Vec<Vec<char>>, h: i32, w: i32, start_pos: (i32, i32), goal_pos: (i32, i32)) -> (i32, u32) {
    // 初期化
    let mut queue: VecDeque<(i32, i32)> = VecDeque::new();
    let mut dist_vec: Vec<Vec<i32>> = Vec::with_capacity(100);
    for row in maze.iter(){
        let mut dist_row_vec: Vec<i32> = Vec::with_capacity(100);
        for _ in 0..row.len() {
            dist_row_vec.push(-1);
        }
        dist_vec.push(dist_row_vec);
    }
    let directions: [(i32, i32); 4]  = [(1, 0), (-1, 0), (0, 1), (0, -1)];

    queue.push_back(start_pos);
    dist_vec[start_pos.0 as usize][start_pos.1 as usize] = 0;
    let mut search_tile_num: u32 = 0;
    while queue.len() > 0 {
        search_tile_num += 1;
        let val = queue.pop_front().unwrap();
        let pos_h = val.0;
        let pos_w = val.1;
        let cost = dist_vec[pos_h as usize][pos_w as usize];
        if pos_h == goal_pos.0 && pos_w == goal_pos.1 {
            break;
        }

        for d in directions {
            let new_h = pos_h + d.0;
            let new_w = pos_w + d.1;
            if 0 <= new_h && new_h < h && 0 <= new_w && new_w < w {
                if maze[new_h as usize][new_w as usize] != '#' && dist_vec[new_h as usize][new_w as usize] == -1 {
                    queue.push_back((new_h, new_w));
                    dist_vec[new_h as usize][new_w as usize] = cost + 1;
                }
            }
        }
    }
    (dist_vec[goal_pos.0 as usize][goal_pos.1 as usize], search_tile_num)
}

fn a_star_bfs(maze: &Vec<Vec<char>>, h: i32, w: i32, start_pos: (i32, i32), goal_pos: (i32, i32)) -> (i32, u32) {

    // 初期化
    let mut heap = BinaryHeap::new();
    let mut dist_vec: Vec<Vec<i32>> = Vec::with_capacity(100);
    for row in maze.iter(){
        let mut dist_row_vec: Vec<i32> = Vec::with_capacity(100);
        for _ in 0..row.len() {
            dist_row_vec.push(-1);
        }
        dist_vec.push(dist_row_vec);
    }
    let directions: [(i32, i32); 4]  = [(1, 0), (-1, 0), (0, 1), (0, -1)];

    heap.push((-heuristic_cost(&start_pos, &goal_pos), start_pos));
    dist_vec[start_pos.0 as usize][start_pos.1 as usize] = 0;
    let mut search_tile_num: u32 = 0;
    while heap.len() > 0 {
        search_tile_num += 1;
        let pop_item = heap.pop().unwrap();
        let pos_h = pop_item.1.0;
        let pos_w = pop_item.1.1;
        let cost = dist_vec[pos_h as usize][pos_w as usize];
        if pos_h == goal_pos.0 && pos_w == goal_pos.1 {
            break;
        }

        for d in directions {
            let new_h = pos_h + d.0;
            let new_w = pos_w + d.1;
            if 0 <= new_h && new_h < h && 0 <= new_w && new_w < w {
                if maze[new_h as usize][new_w as usize] != '#' && dist_vec[new_h as usize][new_w as usize] == -1 {
                    let new_pos = (new_h, new_w);
                    let f_cost = (cost + 1) + heuristic_cost(&new_pos, &goal_pos);
                    heap.push((-f_cost, new_pos));
                    dist_vec[new_h as usize][new_w as usize] = cost + 1;
                }
            }
        }
    }
    (dist_vec[goal_pos.0 as usize][goal_pos.1 as usize], search_tile_num)
}

fn heuristic_cost(p1: &(i32, i32), p2: &(i32, i32)) -> i32 {
    (p1.0 - p2.0).abs() + (p1.1 - p2.1).abs()
}

fn read_maze_from_file(maze_map_path: &str) -> Result<Vec<Vec<char>>, Box<dyn std::error::Error>> {
    let mut maze: Vec<Vec<char>> = Vec::with_capacity(100);    
    for result in BufReader::new(File::open(maze_map_path)?).lines() {
        let mut maze_row: Vec<char> = Vec::with_capacity(100);
        for c in result?.chars() {
            maze_row.push(c);
        }
        maze.push(maze_row);
    }
    Ok(maze)
}

fn find_position(maze: &Vec<Vec<char>>, target: &char) -> Option<(i32, i32)> {
    let mut h: usize = 0;
    for line in maze.iter() {
        let mut w: usize = 0;
        for c in line.iter() {
            if c == target {
                return Some((h as i32, w as i32))
            }
            w += 1;
        }
        h += 1;
    }
    None
}