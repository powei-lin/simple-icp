use crate::config;
use std::{
    fs,
    io::{BufReader, Write},
};
pub fn json_to_config(path: &str) -> config::Config {
    let file = fs::File::open(path).unwrap();
    let reader = BufReader::new(file);
    serde_json::from_reader(reader).unwrap()
}

pub fn write_config_json(path: &str, config: &config::Config) {
    let j = serde_json::to_string_pretty(&config).unwrap();
    let mut file = fs::File::create(path).unwrap();
    file.write_all(j.as_bytes()).unwrap();
}
